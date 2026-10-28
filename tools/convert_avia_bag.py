"""Convert official Livox ROS1 bag to driver2 ROS2, preserving payload/timestamps."""
import argparse,json,hashlib,dataclasses
from pathlib import Path
from rosbags.rosbag1 import Reader
from rosbags.rosbag2 import Writer
from rosbags.typesys import Stores,get_typestore,get_types_from_msg
import numpy as np
p=argparse.ArgumentParser();p.add_argument('source');p.add_argument('destination');p.add_argument('--fast',action='store_true',help='Byte conversion using ROS2 schemas; compare first 20 messages per topic with field conversion');a=p.parse_args()
src=get_typestore(Stores.ROS1_NOETIC);dst=get_typestore(Stores.ROS2_JAZZY)
base=Path(__file__).resolve().parents[1]/'src/livox_ros_driver2/msg'
for name in ['CustomPoint','CustomMsg']:
 dst.register(get_types_from_msg((base/(name+'.msg')).read_text(),'livox_ros_driver2/msg/'+name))
def translate(m):
 if isinstance(m,list):return [translate(x) for x in m]
 if not hasattr(m,'__msgtype__'):return m
 typ=m.__msgtype__.replace('livox_ros_driver/','livox_ros_driver2/')
 cls=dst.types[typ]
 return cls(**{f.name:translate(getattr(m,f.name)) for f in dataclasses.fields(cls) if f.init and not f.name.startswith('__')})
report={'counts':{},'connections':[]};acc=[];offsets=[]
with Reader(Path(a.source)) as r,Writer(Path(a.destination),version=9) as w:
 for c in r.connections:src.register(get_types_from_msg(c.msgdef.data,c.msgtype))
 report.update(duration_s=r.duration/1e9,start_ns=r.start_time,end_ns=r.end_time)
 cs={}
 for c in r.connections:
  report['connections'].append({'topic':c.topic,'type':c.msgtype,'count':c.msgcount})
  if c.topic not in ['/livox/lidar','/livox/imu']:continue
  typ=c.msgtype.replace('livox_ros_driver/','livox_ros_driver2/')
  cs[c.id]=(w.add_connection(c.topic,typ,typestore=dst),typ)
 for c,t,raw in r.messages():
  if c.id not in cs:continue
  conn,typ=cs[c.id]
  if a.fast:
   cdr=dst.ros1_to_cdr(raw,typ)
   if report['counts'].get(c.topic,0)<20:
    reference=dst.serialize_cdr(translate(src.deserialize_ros1(raw,c.msgtype)),typ)
    assert cdr==reference, 'byte conversion differs from field conversion'
  else:
   cdr=dst.serialize_cdr(translate(src.deserialize_ros1(raw,c.msgtype)),typ)
  if a.fast and report['counts'].get(c.topic,0)>=(1000 if c.topic.endswith('imu') else 20):
   w.write(conn,t,cdr);report['counts'][c.topic]+=1;continue
  m=dst.deserialize_cdr(cdr,typ) # validates destination schema before writing
  if c.topic.endswith('imu') and len(acc)<1000:
   v=m.linear_acceleration;acc.append([v.x,v.y,v.z])
  if c.topic.endswith('lidar') and len(offsets)<20:
   offsets.append([m.point_num,len(m.points),min(x.offset_time for x in m.points),max(x.offset_time for x in m.points)])
  w.write(conn,t,cdr);report['counts'][c.topic]=report['counts'].get(c.topic,0)+1
report['first_imu_acc_norm_median']=float(np.median(np.linalg.norm(acc,axis=1)))
report['point_offsets_ns']=offsets
report['sha256']=hashlib.file_digest(open(a.source,'rb'),'sha256').hexdigest()
Path(a.destination).with_suffix('.inspection.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
