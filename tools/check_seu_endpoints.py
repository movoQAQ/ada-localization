"""Independent raw-cloud endpoint registration; no LIO pose used as a seed."""
import argparse,json
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser();p.add_argument('--extract',action='store_true');a=p.parse_args()
root=Path('/workspace/datasets/seu-endpoints')
if a.extract:
 from rosbags.rosbag1 import Reader
 from rosbags.typesys import Stores,get_typestore,get_types_from_msg
 s=get_typestore(Stores.ROS1_NOETIC);meta=[]
 with Reader(Path('/workspace/datasets/seu-library.bag')) as r:
  c=next(c for c in r.connections if c.topic=='/livox/lidar');s.register(get_types_from_msg(c.msgdef.data,c.msgtype))
  chosen={0,10,20,50,c.msgcount-1,c.msgcount-11,c.msgcount-21,c.msgcount-51}
  for i,(_,t,raw) in enumerate(r.messages(connections=[c])):
   if i not in chosen:continue
   m=s.deserialize_ros1(raw,c.msgtype)
   pts=np.array([[q.x,q.y,q.z] for q in m.points if (q.tag&0x30) in (0,0x10)],dtype=np.float64)
   pts=pts[np.isfinite(pts).all(axis=1)];rad=np.linalg.norm(pts,axis=1);pts=pts[(rad>.5)&(rad<80)]
   np.save(root/f'{i}.npy',pts)
   meta.append({'index':i,'header_ns':m.header.stamp.sec*10**9+m.header.stamp.nanosec,'record_ns':t,'points':len(pts)})
 (root/'frames.json').write_text(json.dumps(meta,indent=2));print(meta);raise SystemExit
import open3d as o
reg=o.pipelines.registration
meta=json.loads((root/'frames.json').read_text())
def cloud(i):return o.geometry.PointCloud(o.utility.Vector3dVector(np.load(root/f'{i}.npy')))
def features(c):
 d=c.voxel_down_sample(.5);d.estimate_normals(o.geometry.KDTreeSearchParamHybrid(radius=1.5,max_nn=40))
 return d,reg.compute_fpfh_feature(d,o.geometry.KDTreeSearchParamHybrid(radius=2.5,max_nn=100))
def refine(src,dst,T):
 for v,th in [(0.5,1.5),(.2,.6),(.1,.3)]:
  s=src.voxel_down_sample(v);t=dst.voxel_down_sample(v);t.estimate_normals(o.geometry.KDTreeSearchParamHybrid(radius=v*3,max_nn=40))
  result=reg.registration_icp(s,t,th,T,reg.TransformationEstimationPointToPlane(),reg.ICPConvergenceCriteria(max_iteration=60));T=result.transformation
 return T
def evaluate(s,t,T):
 out={}
 for th in [.2,.5,1.0]:
  f=reg.evaluate_registration(s,t,th,T);b=reg.evaluate_registration(t,s,th,np.linalg.inv(T))
  out[str(th)]={'source_overlap':f.fitness,'target_overlap':b.fitness,'source_rmse':f.inlier_rmse,'target_rmse':b.inlier_rmse}
 return out
results=[]
# Repeat global initialization for endpoint and nearby independent frames.
for si,ti in [(4297,0),(4287,10),(4277,20),(4247,50),(0,4297),(4297,10)]:
 src,dst=cloud(si),cloud(ti);s,sf=features(src);t,tf=features(dst)
 for seed in [11,29,47]:
  o.utility.random.seed(seed)
  coarse=reg.registration_ransac_based_on_feature_matching(s,t,sf,tf,True,1.0,reg.TransformationEstimationPointToPoint(False),3,[reg.CorrespondenceCheckerBasedOnEdgeLength(.9),reg.CorrespondenceCheckerBasedOnDistance(1.0)],reg.RANSACConvergenceCriteria(100000,.999))
  T=refine(src,dst,coarse.transformation)
  row={'source':si,'target':ti,'seed':seed,'distance_m':float(np.linalg.norm(T[:3,3])),'T_target_source':T.tolist(),'metrics':evaluate(src,dst,T),'coarse_fitness':coarse.fitness}
  results.append(row);print(json.dumps(row),flush=True)
  (root/'registration.json').write_text(json.dumps(results,indent=2))
