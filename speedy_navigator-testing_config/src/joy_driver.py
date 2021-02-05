#!/usr/bin/python

import rospy, math
from std_msgs.msg import Int8, Float32
from sensor_msgs.msg import Joy
from speedy_navigator.msg import Floats
import numpy as np
import vehicles

def opmode_callback(msg):
  global op_mode
  op_mode = msg.data

def joy_callback(data):
  global press_tm
  ## drive robot ##
  if op_mode == 3:
    steer_cmd = data.axes[3]*max_steer_ang
    speed_cmd = data.axes[1]*max_speed 
    
    #speed_cmd = 0.5
      
    steer_cmd_pub.publish(steer_cmd)
    speed_cmd_pub.publish(speed_cmd)
   
  new_tm = rospy.get_time()
  if new_tm - press_tm > 0.1:
    press_tm = new_tm
    ## change operation mode ## 
    if data.buttons[2]:
      ui_pub.publish(11)
    elif data.buttons[3]:
      ui_pub.publish(12)
    elif data.buttons[1]:
      ui_pub.publish(13)
    ## start, stop, change speed ##  
    elif data.buttons[0]:
      ui_pub.publish(1)
    elif data.buttons[7]:
      ui_pub.publish(2)
    elif data.buttons[6]:
      ui_pub.publish(3)
    ## shut down main control node ##
    elif data.buttons[4]:
      ui_pub.publish(0)    

if __name__ == '__main__':
  agv = vehicles.Traxxas()
  max_steer_ang, max_speed = agv.max_steer_ang, agv.max_speed
  wheelbase = agv.wheelbase
  op_mode = 1
  
  rospy.init_node('joy_driver', anonymous=True)
  
  steer_rpt = Floats()
  steer_rpt.data = [0, 0]
  
  press_tm = rospy.get_time()

  rospy.Subscriber('/agv/op_mode', Int8, opmode_callback)
  rospy.Subscriber('/joy', Joy, joy_callback)
  steer_cmd_pub = rospy.Publisher('/agv/steer_cmd', Float32, queue_size=1)
  speed_cmd_pub = rospy.Publisher('/agv/speed_cmd', Float32, queue_size=1)
  ui_pub = rospy.Publisher('/agv/keypress', Int8, queue_size=1)

  rospy.spin()
