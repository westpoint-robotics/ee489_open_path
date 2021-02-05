#!/usr/bin/python

import rospy
import numpy as np
from math import pi, sin, cos, atan2
from std_msgs.msg import Bool, Float32
from speedy_navigator.msg import Floats
from rospy.numpy_msg import numpy_msg
import vehicles

def tick_callback(tick):
  global tick_count
  if tick.data:
    tick_count += 1
  else:
    tick_count -= 1
    
### update speed ###
def speed_callback(msg):
  global speed
  speed = msg.data

### reset pose translations and rotations when new LIDAR scan arrives ###
def scan_signal_callback(msg):
  global do_update, delaytm
  do_update = True
  delaytm = rospy.get_time()
  
### continuously update change in pose of the vehicle since scan arrived at first node ###
def update_pose(msg):
  global delta_x, delta_y, delta_ang, steer_ang, turn_radius, pose_ref_tm, pose_ref_ticks
  if do_update:
    if use_speed:
      new_tm = rospy.get_time()
      arc_dist = speed * (new_tm-pose_ref_tm)
      pose_ref_tm = new_tm
    else:
      curr_ticks = tick_count
      arc_dist = meters_per_tick*(curr_ticks - pose_ref_ticks)
      pose_ref_ticks = curr_ticks
    if arc_dist:
      try:
        arc_ang = arc_dist/turn_radius
      except ZeroDivisionError:
        arc_ang = 0
      ## Runge-Kutta method of vehicle motion estimation for the LIDAR sensor location ##
      delta_x += arc_dist*sin(delta_ang + (arc_ang/2) + steer_ang)
      delta_y += arc_dist*cos(delta_ang + (arc_ang/2) + steer_ang)
      delta_ang += arc_ang
  steer_ang, turn_radius = msg.data
    
def correct_paths(msg):
  global corrected, delta_x, delta_y, delta_ang, delaytm
  
  print 'delay time:', rospy.get_time() - delaytm
  
  paths = msg.data.reshape(-1,2,order='C')  
  new_x, new_y = paths[:,1]*np.sin(paths[:,0])-delta_x, paths[:,1]*np.cos(paths[:,0])-delta_y
  corrected_ang = np.arctan2(new_x, new_y) - delta_ang
  delta_x, delta_y, delta_ang = 0.0, 0.0, 0.0
  
  corrected_rng = np.sqrt(new_x**2 + new_y**2)
  corrected.data = np.stack((corrected_ang, corrected_rng), axis=1).flatten(order='C')
  corrected_pub.publish(corrected)


if __name__ == '__main__':  
  rospy.init_node('delay_corrector')
  agv = vehicles.Traxxas()
  
  delaytm = rospy.get_time()
  
  corrected = Floats()
  do_update = False
  steer_ang, turn_radius = 0.0, 0.0
  delta_x, delta_y, delta_ang = 0.0, 0.0, 0.0
  
  pose_ref_tm = rospy.get_time()
  pose_ref_ticks = 0
  
  corrected_pub = rospy.Publisher('/agv/open_paths_corrected', numpy_msg(Floats), queue_size=1)
  use_speed = agv.use_speed
  if use_speed:
    speed = 0 
    rospy.Subscriber('/agv/speed', Float32, speed_callback)
  else:
    meters_per_tick, tick_count = agv.meters_per_tick, 0
    rospy.Subscriber('/agv/tick', Bool, tick_callback)
  rospy.Subscriber('/agv/steer_rpt', Floats, update_pose)
  rospy.Subscriber('/agv/scan_arrived', Bool, scan_signal_callback)
  rospy.Subscriber('/agv/open_paths', numpy_msg(Floats), correct_paths)
  
  rospy.spin()
