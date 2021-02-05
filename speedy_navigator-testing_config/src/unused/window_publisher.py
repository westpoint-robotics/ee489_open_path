#!/usr/bin/python

import rospy
import math
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from obstacle_avoider.srv import PathData, PathDataResponse


### find the rising edges around the aim point ###
def get_boundaries(aim_pt):
  global open_paths, scan, edge
  aim_idx = 0
  try:
    while open_paths[2*aim_idx] < aim_pt:
      aim_idx += 1
  except IndexError:
    aim_idx -= 1
  ## check right ##
  lag_ang = open_paths[2*aim_idx]
  lag_dist = open_paths[2*aim_idx+1]
  if aim_idx > 0:
    lead_idx = aim_idx - 1
    lead_ang = open_paths[2*lead_idx]
    lead_dist = open_paths[2*lead_idx+1]
    while (lag_dist - lead_dist)/(lag_ang - lead_ang) < edge and lead_idx >= 0:
      lag_ang = lead_ang
      lag_dist = lead_dist
      lead_idx -= 1
      lead_ang = open_paths[2*lead_idx]
      lead_dist = open_paths[2*lead_idx+1]
    R_idx = lead_idx + 1
  R_ang = lag_ang
  R_idx = 0
  try:
    while scan[R_idx*2] < R_ang:
      R_idx += 1
  except IndexError:
    R_idx -= 1
  R_dist = lead_dist
  ## check left ##
  lag_ang = open_paths[2*aim_idx]
  lag_dist = open_paths[2*aim_idx+1]
  try:
    lead_idx = aim_idx + 1
    lead_ang = open_paths[2*lead_idx]
    lead_dist = open_paths[2*lead_idx+1]
    while (lag_dist - lead_dist)/(lead_ang - lag_ang) < edge:
      lag_ang = lead_ang
      lag_dist = lead_dist
      lead_idx += 1
      lead_ang = open_paths[2*lead_idx]
      lead_dist = open_paths[2*lead_idx+1]
  except IndexError:
    pass
  L_ang = lag_ang
  L_idx = R_idx
  try:
    while scan[L_idx*2] < L_ang:
      L_idx += 1
  except IndexError:
    L_idx -= 1
  L_idx -= 1
  L_dist = lead_dist
  return (R_idx, R_ang, R_dist, L_idx, L_ang, L_dist)

def open_paths_callback(paths):
  global open_paths
  open_paths = paths.data
  
def scan_callback(msg):
  global scan
  scan = msg.data

def handle_path_data(req):  # [self.aim_steer, self.aim_dist, self.steer_ang, self.speed]
  global open_window
  try:
    window = get_boundaries(req.data[0])
  except IndexError:
    window = (0,0,0,0,0,0)
  open_window.data = window
  pub.publish(open_window)
  return PathDataResponse(window)
  

if __name__ == '__main__':
  open_paths = []
  scan = []
  edge = 12   # minimum sudden change in distance to be considered an edge
  open_window = Float32MultiArray()
  rospy.init_node('window_publisher')
  rospy.Subscriber('/open_paths', Float32MultiArray, open_paths_callback)
  rospy.Subscriber('/scan_transformed', Float32MultiArray, scan_callback)
  pub = rospy.Publisher('/open_window', Float32MultiArray, queue_size=1)
  s = rospy.Service('open_window', PathData, handle_path_data)

  rospy.spin()
