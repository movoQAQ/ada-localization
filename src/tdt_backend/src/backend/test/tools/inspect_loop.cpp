// Offline inspection of a saved runtime-map loop and reconstructed graph.
// Does not alter the saved map or any running backend.
#include "backend/keyframe_map.hpp"
#include "backend/scan_context.hpp"
#include "backend/geometry.hpp"
#include "backend/loop.hpp"
#include "backend/frame_history.hpp"
#include "backend/sampling.hpp"
#include "backend/pose_graph.hpp"
#include <iostream>
#include <fstream>
int main(int argc, char** argv) {
    if (argc < 2) return 2;
    backend::KeyframeMap map;
    if (!map.load(argv[1], 256*1024*1024)) return 3;
    auto frames = map.snapshot();
    if (frames.empty()) return 4;
    const int source_index = argc > 2 ? std::stoi(argv[2]) : frames.back().index;
    const int target_index = argc > 3 ? std::stoi(argv[3]) : 0;
    if(source_index < 0 || source_index >= static_cast<int>(frames.size()) || target_index < 0 || target_index >= source_index) return 4;
    frames.resize(static_cast<std::size_t>(source_index + 1));
    backend::LoopWork work; work.latest=frames.back(); work.trajectory=frames;
    const auto& target=frames.at(static_cast<std::size_t>(target_index));
    backend::ScanContextDescriptor descriptor;
    backend::ScanContextDescriptorConfig sc; sc.distance_threshold=.8; descriptor.configure(sc);
    auto source_cloud=work.latest.cloud.load(),target_cloud=target.cloud.load();
    descriptor.addKeyframe(*backend::transformCloud(*target_cloud,backend::levelFromBodyTransform(target.odom_pose)));
    auto yaw=descriptor.estimateYaw(*backend::transformCloud(*source_cloud,backend::levelFromBodyTransform(work.latest.odom_pose)),0);
    if (!yaw) return 5;
    std::cout << "source=" << source_index << " target=" << target_index << " yaw=" << yaw->yaw_rad << " sc_distance=" << yaw->descriptor_distance << "\n";
    backend::LoopCandidate candidate;candidate.index=target_index;candidate.yaw_seed_rad=yaw->yaw_rad;candidate.descriptor_distance=yaw->descriptor_distance;
    work.candidates={candidate};work.descriptor_level_with_odom_roll_pitch=true;
    backend::LoopConfig lc; lc.place_seed_enabled=true;lc.target_keyframes=17;lc.source.max_frames=4;lc.verification.translation_max=3;lc.verification.z_max=argc > 4 ? std::stod(argv[4]) : .5;lc.verification.roll_pitch_max_rad=10*backend::kPi/180;lc.verification.min_overlap_ratio=.45;
    backend::RegistrationConfig rc;rc.voxel_resolution=.4;rc.max_iterations=16;rc.max_correspondence_distance=1;rc.min_correspondence_ratio=.5;rc.coarse_trigger_min_correspondence_ratio=.35;rc.max_rmse=.25;rc.huber_delta=.25;rc.measurement_translation_std_floor=.005;rc.measurement_rotation_std_floor_rad=.02*backend::kPi/180;
    auto source=backend::accumulateSource(work.latest,frames,lc.source);
    auto sd=backend::voxelize(*source.cloud,.4);
    for (int forward : {0,16}) {
        auto td=backend::voxelize(*backend::LoopMatcher::buildSpatialSubmapFromSnapshot(frames,target_index,17,12,std::min(source_index-4,target_index+forward)),.4);
        backend::Registration matcher(rc);matcher.setTarget(td,target_index);
        Eigen::Matrix4d predicted=target.optimized_pose.inverse()*work.latest.optimized_pose;
        for (bool place : {false,true}) {
            Eigen::Vector3d translation=predicted.block<3,1>(0,3);if(place)translation.setZero();
            auto seed=backend::descriptorYawToRawSourceTarget(yaw->yaw_rad,translation,work.latest.odom_pose,target.odom_pose,true);
            auto r=matcher.align(*sd,seed,.25);
            std::cout << "forward=" << forward << " place=" << place << " target_points=" << td->size() << " source_points=" << sd->size() << " converged=" << r.converged << " geometry=" << r.geometryVerified() << " rmse=" << r.quality.rmse << " ratio=" << r.quality.correspondence_ratio << " sectors=" << r.quality.coverage_sectors << " translation=" << r.source_to_target.block<3,1>(0,3).transpose() << "\n";
        }
    }
    auto result=backend::LoopMatcher({lc,rc}).compute(work);
    std::cout << "selected=" << static_cast<bool>(result.selected) << "\n";
    if (result.selected) {
        for (const std::string kernel : {"cauchy", "huber"}) {
            backend::PoseGraphConfig gc;gc.fixed_lag_seconds=0;gc.robust_kernel=kernel;gc.robust_kernel_size=kernel=="cauchy"?.5:1.;
            gc.odom_noise.rotation_variance_per_second=1e-6;gc.odom_noise.translation_variance_per_second=.0005;
            gc.odom_noise.rotation_variance_per_radian=3e-7;gc.odom_noise.translation_variance_per_meter=.0002;
            backend::PoseGraph graph(gc);graph.addPrior(0,frames[0].odom_pose,frames[0].timestamp);graph.update(false);
            for(std::size_t i=1;i<frames.size();++i) {
                Eigen::Matrix4d delta=frames[i-1].odom_pose.inverse()*frames[i].odom_pose;
                auto covariance=gc.odom_noise.covariance(frames[i].timestamp-frames[i-1].timestamp,delta.block<3,1>(0,3).norm(),backend::rotationAngleFromMatrix(delta));
                graph.addOdometryMeasurement(static_cast<int>(i)-1,static_cast<int>(i),delta,frames[i].odom_pose,covariance,frames[i].timestamp);graph.update(false);
            }
            auto& hit=*result.selected;
            auto term=graph.linearizeRelativePose(source_index,target_index,hit.measured_between,hit.sqrt_information);
            auto assessment=graph.assessInnovationBatch({term});
            std::cout << "offline_graph_no_historical_loops kernel=" << kernel << " NIS=" << assessment.joint.nis << " p=" << assessment.joint.upper_tail_probability << "\n";
            if(assessment.joint.valid && assessment.joint.upper_tail_probability>=.001) {
                graph.addLoopInformation(source_index,target_index,hit.measured_between,hit.sqrt_information);graph.update(true);
                auto last=graph.estimate(source_index),first=graph.estimate(target_index);
                if(last && first)std::cout << "offline_endpoint=" << (last->block<3,1>(0,3)-first->block<3,1>(0,3)).norm() << "\n";
            }
        }
    }

    return 0;
}
