#!/usr/bin/python

import rospy, sys, os
sys.path.append(os.path.dirname(__file__)+'/..')
import vehicles
from math import sqrt, sin, asin, cos
from std_msgs.msg import Int8, Float32
from speedy_navigator.msg import Floats
from sensor_msgs.msg import JointState
from ackermann_msgs.msg import AckermannDrive

def speed_rpt_callback(msg):
  global speed
  speed = msg.data

def speed_cmd_callback(msg):
  global speed_setpoint, cmd
  speed_setpoint = msg.data
  cmd.speed = speed_setpoint
  
def steer_cmd_callback(msg):
  global cmd, steer_rpt
  steer_cmd = msg.data
  if steer_cmd < -max_steer_ang:
    steer_cmd = -max_steer_ang
  elif steer_cmd > max_steer_ang:
    steer_cmd = max_steer_ang
  cmd.steering_angle = steer_cmd  
  try:
    turn_radius = wheelbase/sin(steer_cmd)
  except ZeroDivisionError:
    turn_radius = 0
  steer_rpt.data = [steer_cmd, turn_radius]
  
def get_steer_state(msg):
  steer_ang_left, steer_ang_right = msg.position[4], msg.position[5]
  rad_left, rad_right = wheelbase/sin(steer_ang_left), wheelbase/sin(steer_ang_right)
  x_left = rad_left*cos(steer_ang_left) + track_div_2
  x_right = rad_right*cos(steer_ang_right) - track_div_2
  rad_center_l, rad_center_r = sqrt(x_left**2 + wheelbase**2), sqrt(x_right**2 + wheelbase**2)
  rad_center = max(rad_center_l, rad_center_r)
  if rad_left<0:
    rad_center *= -1
  steer_ang_center = asin(wheelbase/rad_center)
  steering_state_pub.publish([steer_ang_center, rad_center])  
  
def throttle_ramp(speed_err, curr_throttle, pwm_inc, speed_inc):
  print '\nspeed err:', speed_err, ' speed inc:', speed_inc
  if abs(speed_err) < speed_inc:
    inc = 1
  else:
    inc = pwm_inc
  if speed_err > 0:
    out = curr_throttle + inc
  else:
    out = curr_throttle - inc
  if out > max_throttle:
    return max_throttle
  elif out < 0:
    return 0
  else:
    return out
    

if __name__ == '__main__':
  rospy.init_node('servo_interface')

  agv = vehicles.Traxxas()
  wheelbase, track_div_2 = agv.wheelbase, agv.track_div_2
  
  steer_rpt = Floats()
  steer_rpt.data = [0, 0]
  
  cmd = AckermannDrive()
  cmd.steering_angle, cmd.steering_angle_velocity, cmd.speed, cmd.acceleration, cmd.jerk = 0.0, 0.0, 0.0, 0.0, 0.0
  
  max_steer_ang = agv.max_steer_ang
  
  cmd_pub = rospy.Publisher('ackermann_vehicle/ackermann_cmd', AckermannDrive, queue_size=1)
  rospy.Subscriber('/agv/speed_rpt', Float32, speed_rpt_callback)
  rospy.Subscriber('/agv/speed_cmd', Float32, speed_cmd_callback)
  rospy.Subscriber('/agv/steer_cmd', Float32, steer_cmd_callback)
  steer_rpt_pub = rospy.Publisher('/agv/steer_rpt', Floats, queue_size=10)
  steering_state_pub = rospy.Publisher('/agv/true_steer_state', Floats, queue_size=1) # [steer_ang, turn radius]
  rospy.Subscriber('ackermann_vehicle/joint_states', JointState, get_steer_state)
  
  rate = rospy.Rate(agv.control_loop_rate)
  while not rospy.is_shutdown():         
    
    cmd_pub.publish(cmd)
    steer_rpt_pub.publish(steer_rpt)
        
    rate.sleep()
