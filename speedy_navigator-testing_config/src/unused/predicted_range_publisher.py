#!/usr/bin/python

import rospy
import math
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
from obstacle_avoider.srv import PathData, PathDataResponse
import vehicle

### expands the range of the traditional arctan function to be from 1 to 2pi ###
def arctan_expanded(y, x):
  ang = math.atan(y/x)
  if ang>0 and y<0:
    ang -= math.pi
  elif ang<0 and y>0:
    ang += math.pi
  return ang

def adjust_ranges(ranges, speed, tm, steer_ang):
  #print "\n\rspeed: {},  tm: {},  steer ang: {}\r".format(speed,tm,steer_ang)
  #n = 0
  try:
    turn_radius = trax.shaft2shaft/math.sin(steer_ang)
    if steer_ang > 0:
      turn_radius += trax.half_width
    else:
      turn_radius -= trax.half_width
    arc_ang = (speed*tm)/turn_radius
    delta_y = turn_radius*math.sin(arc_ang)
    delta_x = turn_radius*(1 - math.cos(arc_ang))
  except ZeroDivisionError:  # if going straight
    turn_radius = 0
    arc_ang = 0
    delta_y = speed*tm
    delta_x = 0
  #print "turn rad: {},  arc ang: {},  dx: {},  dy: {}\r".format(turn_radius,arc_ang,delta_x,delta_y)
  adjusted_ranges = []
  for idx in range(len(ranges)/2):
    i = 2*idx
    try:
      old_ang = ranges[i]
      old_dist = ranges[i+1]
      comp_ang = old_ang - steer_ang
      new_comp_ang = arctan_expanded(old_dist*math.sin(comp_ang)-delta_x, \
       old_dist*math.cos(comp_ang)-delta_y)
      new_ang = new_comp_ang + steer_ang - arc_ang
    except IndexError:
      pass
    try:
      new_dist = (old_dist*math.sin(comp_ang)-delta_x)/math.sin(new_comp_ang)
    except ZeroDivisionError:
      new_dist = (old_dist*math.cos(comp_ang)-delta_y)/math.cos(new_comp_ang)
    #if new_dist < 0 or new_ang/old_ang < 0:
      #print "old ang: {},  old dist: {}\r".format(old_ang,old_dist)
      #print "comp ang: {},  new comp ang: {}\r".format(comp_ang,new_comp_ang)
      #n += 1
      #print "new ang: {},  new dist: {}\r".format(new_ang,new_dist)
    adjusted_ranges += [new_ang, new_dist]
  return adjusted_ranges

def ranges_callback(msg):
  global ranges
  ranges = msg.data
  
def delay_callback(tm):
  global delay
  delay = tm.data

def handle_path_data(req):  # [self.aim_steer, self.aim_dist, self.steer_ang, self.speed]
  global ranges, prev_steer_ang, prev_speed, delay
  ## account for data processing delay ##
  delay_comp = adjust_ranges(ranges, prev_speed, delay, prev_steer_ang)  
  speed = req.data[3]
  steer_ang = req.data[2]
  ## actual prediction ##
  pred_ranges = adjust_ranges(delay_comp, speed, trax.loop_tm+0.1, steer_ang)
  prev_steer_ang = steer_ang  
  prev_speed = speed
  return PathDataResponse(pred_ranges)
   

if __name__ == '__main__':
  trax = vehicle.Traxxas()
  ranges = []
  delay, prev_steer_ang, prev_speed = 0.0, 0.0, 0.0
  rospy.init_node('predicted_range_publisher')
  rospy.Subscriber('/scan_transformed', Float32MultiArray, ranges_callback)
  rospy.Subscriber('/delay', Float32, delay_callback)
  s = rospy.Service('pred_ranges', PathData, handle_path_data)
  rospy.spin()
