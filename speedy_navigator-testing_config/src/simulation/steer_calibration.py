#!/usr/bin/python

import rospy
from std_msgs.msg import Float32
from speedy_navigator.msg import Floats

def th_steering_callback(msg):
  global rad_th, steer_ang_th
  steer_ang_th = msg.data[2]
  rad_th = msg.data[3]
  
def actual_steering_callback(msg):
  global rad_actual, steer_ang_actual
  steer_ang_actual = msg.data[0]
  rad_actual = msg.data[1]
  
def speed_callback(msg):
  global speed
  speed = msg.data


if __name__ == "__main__":
  speed, rad_th, rad_actual, steer_ang_th, steer_ang_actual = 0.0, 0.0, 0.0, 0.0, 0.0
  rospy.init_node('steer_calibration')
  rospy.Subscriber('/agv/control_rpt', Floats, th_steering_callback)
  rospy.Subscriber('/agv/true_steering_state', Floats, actual_steering_callback)
  rospy.Subscriber('/agv/speed_rpt', Float32, speed_callback)
  
  rate = rospy.Rate(10)
  
  logfile_nm = "/home/user1/catkin_ws/src/speedy_navigator/log/steering_calibration.csv"
  with open(logfile_nm, "a+") as logfile:
    logfile.write("speed, theoretical_rad, actual_rad, theoretical_steer_ang, actual_steer_ang")
      
    while not rospy.is_shutdown():
      if speed:
        logfile.write("\n{}, {}, {}, {}, {}".format(speed, rad_th, rad_actual, steer_ang_th, steer_ang_actual))
    
      rate.sleep()
