import argparse,json,re,collections
from pathlib import Path
import numpy as np
results={}
parser=argparse.ArgumentParser()
parser.add_argument('replays',nargs='+')
parser.add_argument('--optimized-keyframes',help='Saved backend keyframes.txt; requires one replay')
args=parser.parse_args()
if args.optimized_keyframes and len(args.replays)!=1:parser.error('--optimized-keyframes requires one replay')
for arg in args.replays:
 p=Path(arg);s=json.loads((p/'metrics.json').read_text());f=(p/'frontend.log').read_text();b=(p/'backend.log').read_text();rows=[x for x in f.splitlines() if 'lio_quality stamp=' in x]
 d=np.asarray(s['descriptor_pairs']);o=np.asarray(s['odom']);bo=np.asarray(s['backend_odom']);status=collections.Counter(re.search(r'status=([^;]+)',x)[1] for x in s['state'])
 def gaps(x):
  return dict(zip(['p50_ms','p95_ms','p99_ms','max_ms'],map(float,np.percentile(np.diff(x[:,0])/1e6,[50,95,99,100])))) if len(x)>1 else None
 def span(x):return float(np.linalg.norm(x[-1,1:4]-x[0,1:4])) if len(x)>1 else None
 r={'frontend_odometry_count':len(o),'backend_pose_count':len(bo),'cloud_count':s['clouds'],'status_messages':dict(status),'frontend_gaps':gaps(o),'backend_gaps':gaps(bo),'frontend_endpoint_displacement_m':span(o),'backend_endpoint_displacement_m':span(bo),'accepted':sum('accepted=1' in x for x in rows),'rejected':sum('accepted=0' in x for x in rows),'seed':sum('seed=1' in x for x in rows),'accepted_loop_log_count':b.count('loop scan-to-map accepted:'),'fault':('stopped both chains' in f),'node_exit_codes':{k:s[k] for k in s if k.endswith('_exit')},'observer_errors':s['errors'],'backend_node_died':'process has died' in b}
 r['input_imu_count']=len(s.get('input_imu_stamps',[]))
 for label,arr in [('frontend',o),('backend',bo)]:
  if len(arr):
   r[label+'_first_stamp_ns']=int(arr[0,0]);r[label+'_last_stamp_ns']=int(arr[-1,0])
   r[label+'_duration_s']=float((arr[-1,0]-arr[0,0])/1e9)
   r[label+'_endpoint_delta_xyz_m']=(arr[-1,1:4]-arr[0,1:4]).tolist()
 loops=re.findall(r'loop scan-to-map accepted: latest=(\d+), target=(\d+)',b)
 r['accepted_loop_keyframe_pairs']=[[int(x),int(y)] for x,y in loops]
 r['accepted_loop_max_keyframe_gap']=max((abs(int(x)-int(y)) for x,y in loops),default=0)
 r['backend_online_endpoint_displacement_m']=r['backend_endpoint_displacement_m']
 r['endpoint_metric_note']='Online messages are historical estimates, not a retrospectively optimized trajectory. Tail loop optimization can finish after the final IMU.'
 if args.optimized_keyframes:
  k=np.atleast_2d(np.loadtxt(args.optimized_keyframes))
  r['optimized_keyframe_count']=len(k)
  r['optimized_first_stamp_s']=float(k[0,1]);r['optimized_last_stamp_s']=float(k[-1,1])
  r['optimized_endpoint_delta_xyz_m']=(k[-1,2:5]-k[0,2:5]).tolist()
  r['optimized_endpoint_displacement_m']=float(np.linalg.norm(k[-1,2:5]-k[0,2:5]))
 r['quality_counts_are_log_rows']=True
 if len(d):r.update(cropped_point_fraction=float(1-d[:,2].sum()/d[:,1].sum()),full_nonempty_bins_mean=float(d[:,3].mean()),cropped_nonempty_bins_mean=float(d[:,4].mean()),descriptor_changed_fraction=float(np.mean(d[:,5]>1e-6)))
 results[p.name]=r
print(json.dumps(results,indent=2))
