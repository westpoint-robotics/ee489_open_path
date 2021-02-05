#!/usr/bin/python

import rospy, sys
from math import sin
import numpy as np
import vehicles
from std_msgs.msg import Int8, Float32
from speedy_navigator.msg import Floats
sys.path.insert(1, '/home/samuel/ServoHat')
from Adafruit_PWM_Servo_Driver import PWM

def opmode_callback(msg):
  global op_mode
  op_mode = msg.data

def speed_rpt_callback(msg):
  global speed
  speed = msg.data

def speed_cmd_callback(msg):
  global speed_setpoint, throttle_setpoint
  speed_setpoint = msg.data
  throttle_setpoint = speed_setpoint*pwm_speed_ratio
  if throttle_setpoint > max_throttle:
    throttle_setpoint = max_throttle
  elif throttle_setpoint < -max_throttle:
    throttle_setpoint = -max_throttle
  if speed_setpoint: 
    throttle_setpoint = 65
  else:
    throttle_setpoint = 0
  
def steer_cmd_callback(msg):
  global steer_rpt
  steer_ang = msg.data
  if steer_ang > 0.0:
    try:
      cmd = left_pwm_map[np.searchsorted(left_pwm_map[:,0], steer_ang), 1]
    except IndexError:
      cmd = max_left_pwm
      steer_ang = max_steer_ang
  elif steer_ang < 0.0:
    try:
      cmd = right_pwm_map[np.searchsorted(right_pwm_map[:,0], -steer_ang), 1]
    except IndexError:
      cmd = max_right_pwm
      steer_ang = -max_steer_ang
  else:
    cmd = center_pwm
  pwm.setPWM(0, 0, int(cmd))
  try:
    turn_radius = wheelbase/sin(steer_ang)
  except ZeroDivisionError:
    turn_radius = 0
  steer_rpt.data = [steer_ang, turn_radius]
  
def throttle_ramp(current, setpoint, inc):
  if current < setpoint-inc:
    return current + inc
  else:
    return setpoint

def get_steer_center(the_map):
  pre_idx = np.searchsorted(the_map[:,0], 0)
  choices = the_map[[pre_idx-1, pre_idx]]
  choice_idx = np.argmin(np.absolute(choices[:,0]))
  return choices[choice_idx,1]
      

if __name__ == '__main__':
  rospy.init_node('servo_interface')

  agv = vehicles.Traxxas()
  op_mode = 1
  
  ## Initialize PWM ##
  pwm = PWM(0x40)         
  pwm.setPWMFreq(50)
  
  ## Steer control ##
  max_steer_ang, wheelbase = agv.max_steer_ang, agv.wheelbase
  ang_pwm_map = agv.generate_ang_pwm_map()
  left_pwm_map = ang_pwm_map[ang_pwm_map[:,0]>0]  
  right_map_idx = ang_pwm_map[:,0]<0
  right_pwm_map = np.stack((-ang_pwm_map[right_map_idx,0], ang_pwm_map[right_map_idx,1]), axis=1)[::-1]  
  max_right_pwm, max_left_pwm = ang_pwm_map[0,1], ang_pwm_map[-1,1]
  center_pwm = get_steer_center(ang_pwm_map) + 5
  
  steer_rpt = Floats()
  steer_rpt.data = [0, 0]   
  
  ## Speed control ##
  max_throttle, throttle_neutral = agv.max_throttle, agv.throttle_neutral
  speed_setpoint, throttle_setpoint, throttle = 0.0, 0.0, 0.0
  pwm_speed_ratio = agv.pwm_speed_ratio
  
  rospy.Subscriber('/agv/op_mode', Int8, opmode_callback)
  #rospy.Subscriber('/agv/speed', Float32, speed_rpt_callback)
  rospy.Subscriber('/agv/speed_cmd', Float32, speed_cmd_callback)
  rospy.Subscriber('/agv/steer_cmd', Float32, steer_cmd_callback)
  steer_rpt_pub = rospy.Publisher('/agv/steer_rpt', Floats, queue_size=10)
  
  throttle_update_tm = rospy.get_time()
  throttle_update_prd = 0.1
  
  rate = rospy.Rate(agv.control_loop_rate)
  while not rospy.is_shutdown():
  
    steer_rpt_pub.publish(steer_rpt)
  
    if rospy.get_time() - throttle_update_tm > throttle_update_prd:
      throttle_update_tm = rospy.get_time()
      if op_mode == 3:
        throttle = speed_setpoint*pwm_speed_ratio
      else:
        if not speed_setpoint:
          throttle = 0.0
        else:
          throttle = throttle_ramp(throttle, throttle_setpoint, 5)
      
      
      pwm.setPWM(1, 0, int(throttle+throttle_neutral))
        
    rate.sleep()
