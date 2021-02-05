#!/usr/bin/python

import rospy
import numpy as np
from math import pi, sin, cos, atan2
import vehicles
from std_msgs.msg import Bool, Float32
from sensor_msgs.msg import LaserScan
from speedy_navigator.msg import Floats
from rospy.numpy_msg import numpy_msg

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
  
def steer_rpt_callback(msg):
  global delta_x_l, delta_y_l, delta_ang_l, steer_ang_axle, turn_radius_axle, steer_ang_lidar, turn_radius_lidar, pose_ref_tm, pose_ref_ticks  
  if use_speed:
    new_tm = rospy.get_time()
    arc_dist_axle = speed * (new_tm-pose_ref_tm)
    pose_ref_tm = new_tm
  else:
    curr_ticks = tick_count
    arc_dist_axle = meters_per_tick*(curr_ticks - pose_ref_ticks)
    pose_ref_ticks = curr_ticks    
  if arc_dist_axle:
    try:
      arc_ang = arc_dist_axle/turn_radius_axle
      steer_ang_lidar = atan2(turn_radius_axle*sin(steer_ang_axle)-laser2axle, turn_radius_axle*cos(steer_ang_axle))
      turn_radius_lidar = (wheelbase-laser2axle)/sin(steer_ang_lidar)
      delta_x_l, delta_y_l, delta_ang_l = update_pose(delta_x_l, delta_y_l, delta_ang_l, steer_ang_lidar, turn_radius_lidar, arc_ang)
    except ZeroDivisionError:      
      delta_y_l += arc_dist_axle
  steer_ang_axle, turn_radius_axle = msg.data
  
def update_pose(delta_x, delta_y, delta_ang, steer_ang, turn_radius, arc_ang):
  ## The following are the translations and rotations for the LIDAR from scan start to end ##
  delta_x += turn_radius*(cos(steer_ang)-cos(steer_ang+arc_ang))
  delta_y += turn_radius*(sin(steer_ang+arc_ang)-sin(steer_ang))   
  delta_ang += arc_ang
  return delta_x, delta_y, delta_ang

### Function that adjusts the perspective to a point a certain distance forward ###
def translate_fwd(old_ang, old_rng, adj_dist):
  adj_ang = np.arctan2(old_rng*np.sin(old_ang), old_rng*np.cos(old_ang)-adj_dist)
  adj_rng = (old_rng*np.cos(old_ang)-adj_dist) / np.cos(adj_ang)
  return np.stack((adj_ang,adj_rng), axis=1)
  
def scan_callback_no_correction(scan):
  global transformed
  scan_sig_pub.publish(True)
  orig_rng = scan.ranges
  orig_ang = laser_rotation + scan.angle_min + np.arange(orig_rng.size)*scan.angle_increment
  translated = translate_fwd(orig_ang, orig_rng, laser2axle)
  transformed.data = translated[np.argsort(translated[:,0])].flatten(order='C')
  transformed_pub.publish(transformed)

def scan_callback_with_correction(scan):
  global transformed, delta_x_l, delta_y_l, delta_ang_l, pose_ref_tm, pose_ref_ticks
  scan_sig_pub.publish(True)
  orig_rng = scan.ranges
  num_rays = orig_rng.size
  orig_ang = laser_rotation + scan.angle_min + np.arange(num_rays)*scan.angle_increment
  ## update pose again ##
  if use_speed:
    new_tm = rospy.get_time()
    arc_dist_axle = speed * (new_tm-pose_ref_tm)
    pose_ref_tm = new_tm
  else:
    curr_ticks = tick_count
    arc_dist_axle = meters_per_tick*(curr_ticks - pose_ref_ticks)
    pose_ref_ticks = curr_ticks
  if arc_dist_axle:
    try:
      arc_ang = arc_dist_axle/turn_radius_axle
    except ZeroDivisionError:
      arc_ang = 0
    try:
      arc_dist_lidar = arc_dist_axle*(turn_radius_lidar/turn_radius_axle)
    except ZeroDivisionError:
      arc_dist_lidar = arc_dist_axle
    delta_x_l, delta_y_l, delta_ang_l = update_pose(delta_x_l, delta_y_l, delta_ang_l, steer_ang_lidar, arc_ang, arc_dist_lidar)
  ## assume linear change in position and orientation with respect to time ##
  ray_idx = np.arange(num_rays)
  ray_frac = ((orig_ang[-1]-orig_ang[0])/(2*pi))*(num_rays-ray_idx-1)/num_rays
  delta_x_ray, delta_y_ray, delta_ang_ray = delta_x_l*ray_frac, delta_y_l*ray_frac, delta_ang_l*ray_frac  
  delta_x_l, delta_y_l, delta_ang_l = 0.0, 0.0, 0.0  
  ## correct the ranges ##
  new_x, new_y = orig_rng*np.sin(orig_ang)-delta_x_ray, orig_rng*np.cos(orig_ang)-delta_y_ray
  corrected_ang = np.arctan2(new_x, new_y) - delta_ang_ray
  corrected_rng = np.sqrt(new_x**2 + new_y**2)
  translated = translate_fwd(corrected_ang, corrected_rng, laser2axle)
  transformed.data = translated[np.argsort(translated[:,0])].flatten(order='C')
  transformed_pub.publish(transformed)


if __name__ == '__main__':
  agv = vehicles.Traxxas()
  wheelbase, laser2axle, laser_rotation = agv.wheelbase, agv.laser2axle, agv.laser_rotation  
  transformed = Floats()
  
  rospy.init_node('scan_transformer')
  
  scan_sig_pub = rospy.Publisher('/agv/scan_arrived', Bool, queue_size=1)
  transformed_pub = rospy.Publisher('/agv/scan_transformed', numpy_msg(Floats), queue_size=1)   
  
  if agv.do_correct_distortion:    
    steer_ang_axle, turn_radius_axle = 0.0, 0.0
    steer_ang_lidar, turn_radius_lidar = 0.0, 0.0
    delta_x_l, delta_y_l, delta_ang_l = 0.0, 0.0, 0.0    
    pose_ref_tm = rospy.get_time()
    pose_ref_ticks = 0
    use_speed = agv.use_speed
    if use_speed:
      speed = 0 
      rospy.Subscriber('/agv/speed', Float32, speed_callback)
    else:
      meters_per_tick, tick_count = agv.meters_per_tick, 0
      rospy.Subscriber('/agv/tick', Bool, tick_callback)
    rospy.Subscriber('/agv/steer_rpt', Floats, steer_rpt_callback)
    rospy.Subscriber('/scan', numpy_msg(LaserScan), scan_callback_with_correction)
  else:
    rospy.Subscriber('/scan', numpy_msg(LaserScan), scan_callback_no_correction)  
  
  rospy.spin()
