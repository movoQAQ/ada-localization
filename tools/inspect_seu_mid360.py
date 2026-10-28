"""Inspect the SEU ROS1 Mid-360 bag without changing its sensor data."""
import argparse,json,hashlib
from pathlib import Path
import numpy as np
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores,get_typestore,get_types_from_msg
parser=argparse.ArgumentParser()
parser.add_argument('source',type=Path)
parser.add_argument('--output-prefix',type=Path,required=True)
args=parser.parse_args();path=args.source
s=get_typestore(Stores.ROS1_NOETIC);out={};imus=[];scans=[]
with Reader(path) as r:
 out['duration_s']=r.duration/1e9
 out['connections']=[{'topic':c.topic,'type':c.msgtype,'count':c.msgcount} for c in r.connections]
 for c in r.connections:s.register(get_types_from_msg(c.msgdef.data,c.msgtype))
 for c,t,raw in r.messages():
  if c.msgtype=='sensor_msgs/msg/Imu':
   m=s.deserialize_ros1(raw,c.msgtype);a=m.linear_acceleration;g=m.angular_velocity
   imus.append([m.header.stamp.sec+m.header.stamp.nanosec/1e9,a.x,a.y,a.z,g.x,g.y,g.z,t/1e9])
  elif c.msgtype.endswith('/CustomMsg') and len(scans)<20:
   m=s.deserialize_ros1(raw,c.msgtype)
   scans.append({'topic':c.topic,'frame':m.header.frame_id,'stamp':m.header.stamp.sec+m.header.stamp.nanosec/1e9,'timebase':int(m.timebase),'points':len(m.points),'offset_range':[int(min(p.offset_time for p in m.points)),int(max(p.offset_time for p in m.points))]})
a=np.array(imus);first=a[a[:,0]<a[0,0]+2]
out['first_2s']={'samples':len(first),'acc_norm_median':float(np.median(np.linalg.norm(first[:,1:4],axis=1))),'acc_var':np.var(first[:,1:4],axis=0).tolist(),'gyro_norm_max':float(np.max(np.linalg.norm(first[:,4:7],axis=1))),'gyro_var':np.var(first[:,4:7],axis=0).tolist()}
d=np.diff(a[:,0]);out['imu_timing']={'count':len(a),'nonpositive_intervals':int((d<=0).sum()),'max_gap_s':float(d.max()),'p99_gap_s':float(np.quantile(d,.99)),'record_header_offset_median_s':float(np.median(a[:,7]-a[:,0]))}
out['scans']=scans
out['sha256']=hashlib.file_digest(path.open('rb'),'sha256').hexdigest()
Path(str(args.output_prefix)+'-inspection.json').write_text(json.dumps(out,indent=2));np.save(str(args.output_prefix)+'-imu.npy',a)
print(json.dumps(out,indent=2))
