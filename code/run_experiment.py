#!/usr/bin/env python3
import csv,json,math,time,threading
from pathlib import Path
from gz.transport13 import Node
from gz.msgs10.odometry_pb2 import Odometry
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.twist_pb2 import Twist
P=Path(__file__).resolve().parents[1]
node=Node(); lock=threading.Lock(); od={}; gt={}; latest=None

def stamp(m): return round(m.header.stamp.sec+m.header.stamp.nsec*1e-9,6)
def pose(p):
 q=p.orientation
 return [p.position.x,p.position.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]
def oc(m):
 global latest
 with lock: od[stamp(m)]=pose(m.pose); latest=(stamp(m),pose(m.pose))
def gc(m):
 with lock:
  for p in m.pose:
   if p.name=='vehicle_blue': gt[stamp(p)]=pose(p)
assert node.subscribe(Odometry,'/model/vehicle_blue/odometry',oc)
assert node.subscribe(Pose_V,'/model/vehicle_blue/pose',gc)
pub=node.advertise('/model/vehicle_blue/cmd_vel',Twist)
def send(v=0,w=0):
 m=Twist();m.linear.x=v;m.angular.z=w;pub.publish(m)
def wrap(a):return math.atan2(math.sin(a),math.cos(a))
def state():
 with lock:return latest
start=time.monotonic()
while state() is None or not gt:
 if time.monotonic()-start>20:raise RuntimeError('Missing streams')
 time.sleep(.05)
send();time.sleep(1)
t0,initial=state(); phases=[]
def drive(label,dist=None,angle=None):
 phases.append({'phase':label,'start_sim_s':state()[0]})
 _,p0=state(); deadline=time.monotonic()+45
 while True:
  _,p=state()
  e=dist-math.hypot(p[0]-p0[0],p[1]-p0[1]) if dist is not None else angle-wrap(p[2]-p0[2])
  if abs(e)<(.003 if dist is not None else .005):break
  if time.monotonic()>deadline:raise RuntimeError('Trajectory timeout')
  send(max(-.3,min(.3,1.5*e)) if dist is not None else 0, max(-.5,min(.5,1.5*e)) if angle is not None else 0)
  time.sleep(.02)
 send();time.sleep(1)
 print(label,state(),flush=True)
try:
 drive('straight_2m',dist=2);drive('left_turn_90deg',angle=math.pi/2);drive('straight_1m',dist=1);send();time.sleep(2)
finally:send()
t1=state()[0]
with lock: od=dict(od);gt=dict(gt)
# Compare at identical simulator timestamps; preserve global ground truth.
keys=sorted(set(od)&set(gt));keys=[t for t in keys if t0<=t<=t1]
if len(keys)<100:raise RuntimeError(f'Insufficient paired samples: {len(keys)}')
o0=od[keys[0]];g0=gt[keys[0]];rotation=g0[2]-o0[2]
rows=[]
for t in keys:
 o=od[t];g=gt[t];dx=o[0]-o0[0];dy=o[1]-o0[1]
 x=g0[0]+math.cos(rotation)*dx-math.sin(rotation)*dy;y=g0[1]+math.sin(rotation)*dx+math.cos(rotation)*dy
 # Wheel odometry tracks the axle midpoint; convert to the model origin.
 x+=0.5542825*(math.cos(g0[2])-math.cos(o[2]+rotation))
 y+=0.5542825*(math.sin(g0[2])-math.sin(o[2]+rotation))
 rows.append([t-t0,t,*o,*g,x,y,math.hypot(x-g[0],y-g[1]),wrap(o[2]+rotation-g[2])])
with (P/'data/trajectory.csv').open('w') as f:
 w=csv.writer(f);w.writerow(['time_s','sim_time_s','odom_x','odom_y','odom_yaw','gt_x','gt_y','gt_yaw','aligned_odom_x','aligned_odom_y','error_m','yaw_error_rad']);w.writerows(rows)
summary={'samples':len(rows),'duration_s':rows[-1][0],'final_position_error_m':rows[-1][-2],'final_yaw_error_rad':rows[-1][-1],'rmse_m':math.sqrt(sum(r[-2]**2 for r in rows)/len(rows)),'max_error_m':max(r[-2] for r in rows),'initial_world_pose':g0,'phases':phases,'timestamp_pairing':'exact simulation timestamp','alignment':'one fixed initial SE(2) transform, evaluation only'}
(P/'results/metrics.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))
