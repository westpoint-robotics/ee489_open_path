#!/usr/bin/python

import rospy
import math
import numpy as np
from std_msgs.msg import Int8
from speedy_navigator.msg import Floats
from rospy.numpy_msg import numpy_msg
import vehicles

def opmode_callback(msg):
  global op_mode
  op_mode = msg.data

def limits_callback(msg):
  global limits
  right, left = msg.data[0]+cushion_B, msg.data[2]-cushion_B
  if op_mode==1 and right>-max_steer_ang:
    limits[0] = -max_steer_ang
  elif right < -hardlimit:
    limits[0] = -hardlimit
  else:
    limits[0] = right
  if op_mode==1 and left<max_steer_ang:
    limits[1] = max_steer_ang
  elif left > hardlimit:
    limits[1] = hardlimit
  else:
    limits[1] = left

### Function that generates look up dictionary indexed by angular half width offset and
### containing required range before obstruction
def generate_req_dist_lookup(inc_ang, max_calc_ang, max_ang, half_width):
  lookup = [max_scan_rng+1]
  for idx in range(1, int(max_ang//inc_ang)+1):
    off_ang = idx*inc_ang
    if off_ang < max_calc_ang:
      req_dist = half_width / math.sin(off_ang)
    else:
      req_dist = 0
    lookup.append(req_dist)
  return np.array(lookup, order='C')

### Function to find the distance open in each steerable direction ###
### Takes 2 dimensional array of angle and range pairs (at equal angular intervals) as input ###
def search_open(points, headings, inc_ang, req_dist_lookup, max_scan_rng):
  pt_angs, pt_rngs = points[:,0][:,np.newaxis], points[:,1][:,np.newaxis]
  offset_ang = np.absolute(pt_angs - headings)
  req_dist = req_dist_lookup[np.floor_divide(offset_ang, inc_ang).astype(np.intp)]
  open_rngs = np.amin(np.where(pt_rngs<req_dist, np.absolute(pt_rngs*np.cos(offset_ang)), max_scan_rng), axis=0) - axle2front
  return np.stack((headings, open_rngs), axis=1)  
  
def scan_callback(msg):
  global open_paths  
  headings = heading_lookup[(heading_lookup>limits[0]) & (heading_lookup<limits[1])]
  scan = msg.data.reshape(-1,2,order='C')
  try:
    open_paths.data = search_open(scan[(scan[:,0]>limits[0]-ang_to_corner) & (scan[:,0]<limits[1]+ang_to_corner)], headings, scan_res, req_dist_lookup, max_scan_rng).flatten(order='C')
  except ValueError:
    open_paths.data = [0.0, 0.0]
  pub.publish(open_paths)


if __name__ == '__main__':
  open_paths = Floats()	# data type to be published  
  agv = vehicles.Traxxas()
  op_mode = 1
  
  ang_to_corner, axle2front, max_scan_rng, scan_res = agv.ang_to_corner, agv.axle2front, agv.max_scan_rng, agv.scan_res
  max_steer_ang = agv.max_steer_ang
  hardlimit = agv.max_tgt_ang # limits on angular range to calculate open paths   
  cushion_B = agv.get_cushion(max_steer_ang)[0]
  limits = [-hardlimit, hardlimit]
  
  heading_lookup = np.linspace(-hardlimit, hardlimit, 2*hardlimit/agv.steer_res)
  req_dist_lookup = generate_req_dist_lookup(scan_res, ang_to_corner, 2*math.pi, agv.half_width)
  
  rospy.init_node('open_paths_publisher')
  rospy.Subscriber('/agv/op_mode', Int8, opmode_callback)
  
  pub = rospy.Publisher('/agv/open_paths', numpy_msg(Floats), queue_size=1)
  rospy.Subscriber('/agv/turn_limits', Floats, limits_callback)
  rospy.Subscriber('/agv/scan_filtered', numpy_msg(Floats), scan_callback)
  rospy.spin()
