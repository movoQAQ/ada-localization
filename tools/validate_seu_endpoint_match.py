"""Validate endpoint registration on held-out points and save projections."""
from pathlib import Path
import json
import numpy as np
import open3d as o
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root=Path('/workspace/datasets/seu-endpoints');reg=o.pipelines.registration
s=np.load(root/'4297.npy');t=np.load(root/'0.npy')
def pc(x):return o.geometry.PointCloud(o.utility.Vector3dVector(x))
rng=np.random.default_rng(83);si=rng.permutation(len(s));ti=rng.permutation(len(t))
train_s,hold_s=pc(s[si[::2]]),pc(s[si[1::2]])
train_t,hold_t=pc(t[ti[::2]]),pc(t[ti[1::2]])
def prep(c):
 d=c.voxel_down_sample(.5);d.estimate_normals(o.geometry.KDTreeSearchParamHybrid(radius=1.5,max_nn=40));return d,reg.compute_fpfh_feature(d,o.geometry.KDTreeSearchParamHybrid(radius=2.5,max_nn=100))
a,af=prep(train_s);b,bf=prep(train_t);o.utility.random.seed(83)
r=reg.registration_ransac_based_on_feature_matching(a,b,af,bf,True,1.,reg.TransformationEstimationPointToPoint(False),3,[reg.CorrespondenceCheckerBasedOnEdgeLength(.9),reg.CorrespondenceCheckerBasedOnDistance(1.)],reg.RANSACConvergenceCriteria(200000,.999))
T=r.transformation
for v,th in [(.5,1.5),(.2,.6),(.1,.3)]:
 a=train_s.voxel_down_sample(v);b=train_t.voxel_down_sample(v);b.estimate_normals(o.geometry.KDTreeSearchParamHybrid(radius=3*v,max_nn=40));T=reg.registration_icp(a,b,th,T,reg.TransformationEstimationPointToPlane(),reg.ICPConvergenceCriteria(max_iteration=60)).transformation
out={'heldout_distance_m':float(np.linalg.norm(T[:3,3])),'T_target_source':T.tolist(),'heldout_metrics':{}}
for th in [.2,.5,1.]:
 f=reg.evaluate_registration(hold_s,hold_t,th,T);b=reg.evaluate_registration(hold_t,hold_s,th,np.linalg.inv(T));out['heldout_metrics'][str(th)]={'source_overlap':f.fitness,'target_overlap':b.fitness,'source_rmse':f.inlier_rmse,'target_rmse':b.inlier_rmse}
(root/'heldout.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
q=s@T[:3,:3].T+T[:3,3]
fig,axes=plt.subplots(1,3,figsize=(15,5))
for ax,(i,j),title in zip(axes,[(0,1),(0,2),(1,2)],['XY','XZ','YZ']):
 ax.scatter(t[:,i],t[:,j],s=1,alpha=.3,c='#0072B2',label='First raw scan')
 ax.scatter(q[:,i],q[:,j],s=1,alpha=.3,c='#D55E00',label='Last raw scan, independently registered')
 ax.set(xlabel='XYZ'[i]+' (m)',ylabel='XYZ'[j]+' (m)',title=title,xlim=(-30,30),ylim=(-30,30));ax.set_aspect('equal');ax.grid(alpha=.2)
axes[0].legend(fontsize=7,loc='lower left',markerscale=5)
fig.suptitle(f'SEU library raw endpoints: held-out-fit translation {out["heldout_distance_m"]:.3f} m (no LIO initialization)')
fig.tight_layout();fig.savefig(root/'endpoint-overlap.png',dpi=160)
