#!/usr/bin/python

import rospy
from math import sin, cos, atan2, sqrt
import vehicles
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import PoseStamped
from speedy_navigator.msg import Floats, WayPoint

def tick_callback(tick):
  global tick_count
  if tick.data:
    tick_count += 1
  else:
    tick_count -= 1

def nav_goal_callback(msg):
  global dest, dest_is_new
  x, y = msg.pose.position.x, msg.pose.position.y
  ## in RVIZ, positive x is forward and positive y is left
  dest.heading = atan2(y,x)
  dest.range = sqrt(x**2 + y**2)
  dest_is_new = True
    
### update speed ###
def speed_callback(msg):
  global speed
  speed = msg.data
  
def steer_params_callback(msg):
  global dest, steer_ang, turn_radius, ref_ticks, ref_tm
  if use_speed:
    new_tm = rospy.get_time()
    dist_traveled = speed * (new_tm-ref_tm)
    ref_tm = new_tm 
  else:
    curr_ticks = tick_count
    dist_traveled = meters_per_tick*(curr_ticks - ref_ticks)
    ref_ticks = curr_ticks       
  if dist_traveled:  
    dest.heading, dest.range = update_point(dest.heading, dest.range, steer_ang, turn_radius, dist_traveled)
  steer_ang, turn_radius = msg.data
    
def update_point(old_ang, old_rng, steer_ang, turn_radius, dist): 
  try:
    arc_ang = dist/turn_radius  # positive is counterclockwise    
    delta_x = turn_radius*(cos(steer_ang)-cos(steer_ang+arc_ang))
    delta_y = turn_radius*(sin(steer_ang+arc_ang)-sin(steer_ang))    
  except ZeroDivisionError:
    arc_ang = 0
    delta_x, delta_y = 0, dist
  new_x, new_y = old_rng*sin(old_ang)-delta_x, old_rng*cos(old_ang)-delta_y
  theta = atan2(new_x, new_y)
  new_ang = theta - arc_ang
  new_rng = sqrt(new_x**2 + new_y**2)
  return new_ang, new_rng
    
  
if __name__ == '__main__':
  dest = WayPoint()
  dest.heading, dest.range = 0.0, 0.0
  dest_is_new = False  
  agv = vehicles.Traxxas()
  use_speed = agv.use_speed
  
  rospy.init_node('localizer')
  
  ref_ticks, ref_tm = 0, rospy.get_time()
  steer_ang, turn_radius = 0.0, 0.0  
  
  dest_pub = rospy.Publisher('/agv/destination', WayPoint, queue_size=1)
  rospy.Subscriber('/move_base_simple/goal', PoseStamped, nav_goal_callback)
  if use_speed:
    speed = 0 
    rospy.Subscriber('/agv/speed', Float32, speed_callback)
  else:
    meters_per_tick, tick_count = agv.meters_per_tick, 0
    rospy.Subscriber('/agv/tick', Bool, tick_callback)  
  rospy.Subscriber('/agv/steer_rpt', Floats, steer_params_callback)
  
  rate = rospy.Rate(30)
  while not rospy.is_shutdown():
    if dest_is_new:
      dest.is_new = True
      dest_is_new = False
    else:
      dest.is_new = False
    dest_pub.publish(dest)
    rate.sleep()
