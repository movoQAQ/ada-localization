"""Use sensor Header time as ROS2 playback time, leaving message bytes unchanged.
Original recording timestamps contain delivery bursts; original bag is retained.
"""
import pathlib,shutil,sqlite3,yaml,struct
base=pathlib.Path('/workspace/datasets');src=base/'quick-shack-ros2-v2';dst=base/'quick-shack-sensor-clock'
shutil.copytree(src,dst)
p=dst/'metadata.yaml';m=yaml.safe_load(p.read_text());info=m['rosbag2_bagfile_information'];stamps=[]
for db in dst.glob('*.db3'):
 with sqlite3.connect(db) as c:
  updates=[]
  for i,data in c.execute('SELECT id,data FROM messages'):
   assert data[:2]==b'\x00\x01', 'expected little-endian CDR'
   sec,ns=struct.unpack_from('<iI',data,4);assert sec>=0 and ns<10**9
   stamp=sec*10**9+ns;updates.append((stamp,i));stamps.append(stamp)
  c.executemany('UPDATE messages SET timestamp=? WHERE id=?',updates)
start,end=min(stamps),max(stamps)
info['starting_time']['nanoseconds_since_epoch']=start;info['duration']['nanoseconds']=end-start
for f in info.get('files',[]):
 f['starting_time']['nanoseconds_since_epoch']=start;f['duration']['nanoseconds']=end-start
p.write_text(yaml.safe_dump(m,sort_keys=False));(dst/'clock_alignment.txt').write_text('Record timestamps set to unchanged message Header time; raw recording retained. Delivery-burst timing is not reproduced.\n')
print('sensor-clock duration', (end-start)/1e9)
