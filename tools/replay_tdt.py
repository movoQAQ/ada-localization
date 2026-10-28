"""Joint real-data replay; reports behavior, never ground-truth accuracy."""
import argparse,json,os,signal,subprocess,time,re
from pathlib import Path
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import PointCloud2,Imu
from sensor_msgs_py.point_cloud2 import read_points_numpy,read_points,create_cloud
from std_msgs.msg import String
p=argparse.ArgumentParser();p.add_argument('--bag',required=True);p.add_argument('--config',required=True);p.add_argument('--output',required=True);p.add_argument('--rate',default='1');p.add_argument('--crop',action='store_true');p.add_argument('--duration',type=float,default=-1);p.add_argument('--delay',type=float,default=2);p.add_argument('--backend-config',default='/workspace/Adaptive-LIO/src/Adaptive-LIO/config/tdt_backend.yaml');a=p.parse_args()
out=Path(a.output);out.mkdir(parents=True,exist_ok=False)
(out/'frontend-config.yaml').write_text(Path(a.config).read_text())
(out/'ros.yaml').write_text('/**:\n  ros__parameters:\n    use_sim_time: true\n    publish_backend_frames: true\n')
rclpy.init();n=rclpy.create_node('tdt_replay_observer');qos=qos_profile_sensor_data
relay=n.create_publisher(PointCloud2,'/adaptive_lio/backend/cloud_cropped',qos)
backend_config=a.backend_config
(out/'backend-config.yaml').write_text(Path(backend_config).read_text())
# Freeze inherited configuration too: a leaf extends path is not a reproducible snapshot.
import yaml

def resolved_config(path, stack=()):
 path=Path(path).resolve()
 if path in stack:raise ValueError('cyclic backend configuration')
 cfg=yaml.safe_load(path.read_text()) or {}
 parent=cfg.pop('extends',None)
 base={}
 if parent:
  if not isinstance(parent,str):raise ValueError('expected a single extends path')
  base=resolved_config(path.parent/parent,stack+(path,))
 def merge(dst,src):
  for key,value in src.items():
   if isinstance(value,dict) and isinstance(dst.get(key),dict):merge(dst[key],value)
   else:dst[key]=value
 merge(base,cfg)
 return base
(out/'backend-effective.yaml').write_text(yaml.safe_dump(resolved_config(backend_config),sort_keys=False))

if a.crop:
 import yaml
 path=out/'cropped_backend.yaml';path.write_text(yaml.safe_dump({'extends':backend_config,'common':{'cloud_topic':'/adaptive_lio/backend/cloud_cropped'}}));backend_config=str(path.resolve())
stats={'odom':[],'backend_odom':[],'state':[],'clouds':0,'descriptor_pairs':[],'errors':[],'input_imu_stamps':[]};poses={};clouds={};descs=[]
def stamp(m):return m.header.stamp.sec*10**9+m.header.stamp.nanosec
def odom(m,key):
 p=m.pose.pose.position;q=m.pose.pose.orientation
 row=[stamp(m),p.x,p.y,p.z,q.x,q.y,q.z,q.w]
 if not np.isfinite(row).all():stats['errors'].append(key+' nonfinite')
 if stats[key] and row[0]<=stats[key][-1][0]:stats['errors'].append(key+' nonmonotonic')
 stats[key].append(row)
def pair(t):
 if t not in poses or t not in clouds:return
 m=poses.pop(t);w=clouds.pop(t);p=m.pose.pose.position;q=m.pose.pose.orientation
 x,y,z,h=q.x,q.y,q.z,q.w
 R=np.array([[1-2*(y*y+z*z),2*(x*y-z*h),2*(x*z+y*h)],[2*(x*y+z*h),1-2*(x*x+z*z),2*(y*z-x*h)],[2*(x*z-y*h),2*(y*z+x*h),1-2*(x*x+y*y)]])
 keep=(w[:,2]>=-1)&(w[:,2]<=2);local=(w-np.array([p.x,p.y,p.z]))@R
 def desc(points):
  d=np.full((20,60),-np.inf);r=np.linalg.norm(points[:,:2],axis=1);v=np.isfinite(points).all(axis=1)&(r<80)
  r=r[v];b=points[v];i=np.minimum((r/4).astype(int),19);j=np.minimum(((np.arctan2(b[:,1],b[:,0])%(2*np.pi))*60/(2*np.pi)).astype(int),59)
  np.maximum.at(d,(i,j),b[:,2]);d[~np.isfinite(d)]=0;return d
 f,c=desc(local),desc(local[keep]);stats['descriptor_pairs'].append([t,len(w),int(keep.sum()),int(np.count_nonzero(f)),int(np.count_nonzero(c)),float(np.linalg.norm(f-c))]);descs.append([f,c])
def frame(m):
 t=stamp(m);poses[t]=m;pair(t)
def cloud(m):
 stats['clouds']+=1;t=stamp(m)
 if a.crop:
  pts=read_points(m);pts=pts[(pts['z']>=-1)&(pts['z']<=2)];relay.publish(create_cloud(m.header,m.fields,pts,point_step=m.point_step))
 w=read_points_numpy(m,field_names=['x','y','z'],skip_nans=True).reshape(-1,3);clouds[t]=w;pair(t)
 # Keep unmatched data bounded.
 for d in [poses,clouds]:
  while len(d)>50:d.pop(next(iter(d)))
n.create_subscription(Imu,'/livox/imu',lambda m:stats['input_imu_stamps'].append(stamp(m)),500)
n.create_subscription(Odometry,'/odom',lambda m:odom(m,'odom'),qos)
n.create_subscription(PoseWithCovarianceStamped,'/backend_node/localization_pose',lambda m:odom(m,'backend_odom'),qos)
n.create_subscription(Odometry,'/adaptive_lio/backend/odometry',frame,qos)
n.create_subscription(PointCloud2,'/adaptive_lio/backend/cloud',cloud,qos)
n.create_subscription(String,'/backend_node/localization_state',lambda m:stats['state'].append(m.data),10)
children=[];logs=[]
def start(cmd,name):
 f=(out/(name+'.log')).open('w');logs.append(f);s=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT);children.append((name,s));return s
try:
 front=start(['/workspace/adaptive-lio-env/install/adaptive_lio/lib/adaptive_lio/adaptive_lio','--config_file='+a.config,'--ros-args','--params-file',str((out/'ros.yaml').resolve())],'frontend')
 back=start(['ros2','launch','backend','backend.launch.py','config_file:='+backend_config,'mode:=mapping','use_sim_time:=true','rviz:=false'],'backend')
 deadline=time.monotonic()+4
 while time.monotonic()<deadline:rclpy.spin_once(n,timeout_sec=.05)
 player=start(['ros2','bag','play',a.bag,'--clock','100','--rate',a.rate,'--delay',str(a.delay),'--playback-duration',str(a.duration),'--disable-keyboard-controls'],'player')
 deadline=time.monotonic()+600
 while player.poll() is None and time.monotonic()<deadline:
  rclpy.spin_once(n,timeout_sec=.01)
  if front.poll() is not None or back.poll() is not None:raise RuntimeError('node exited during playback')
 if player.poll() is None:raise RuntimeError('playback timeout')
 if player.returncode:raise RuntimeError('player failed')
 deadline=time.monotonic()+5
 while time.monotonic()<deadline:rclpy.spin_once(n,timeout_sec=.05)
except KeyboardInterrupt:
 stats['errors'].append('observer interrupted; incomplete replay')
finally:
 for name,s in reversed(children):
  if s.poll() is None:
   s.send_signal(signal.SIGINT)
   try:s.wait(timeout=20)
   except subprocess.TimeoutExpired:s.kill();s.wait();stats['errors'].append(name+' forced shutdown')
  stats[name+'_exit']=s.returncode
 for f in logs:f.close()
 for name in ['frontend','backend']:
  text=(out/(name+'.log')).read_text()
  if 'stopped both chains' in text or 'process has died' in text:stats['errors'].append(name+' fault/crash')
 if not stats['backend_odom']:stats['errors'].append('no backend localization output')
 n.destroy_node();rclpy.try_shutdown()
 (out/'metrics.json').write_text(json.dumps(stats,indent=2));np.savez_compressed(out/'descriptors.npz',pairs=np.asarray(descs))
print(json.dumps({k:len(v) if isinstance(v,list) else v for k,v in stats.items()},indent=2))
if not stats['odom'] or not stats['clouds'] or stats['errors']:raise SystemExit(1)
