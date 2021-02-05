#!/usr/bin/python

import rospy, os, math
from std_msgs.msg import Float32
from speedy_navigator.msg import Floats

def speed_callback(msg):
  global speed
  speed = msg.data
  
def tgt_callback(msg):
  global tgt_heading, tgt_rng
  tgt_heading, tgt_rng = msg.data

def steer_rpt_callback(msg):
  global logfile
  steer_ang, turn_radius = msg.data
  try:
    logfile.write(str(rospy.get_time()-start_tm)+str(tgt_heading)+str(tgt_rng)+str(steer_ang)+str(turn_radius)+str(speed)+'\n')
  except ValueError:  # In case log file is already closed
    pass

def shutdown_hook(self):
  logfile.close()

if __name__ == '__main__':
  speed, tgt_heading, tgt_rng = 0, 0, 0

  rospy.init_node('data_logger')
  logfile_nm = "/home/user1/catkin_ws/src/speedy_navigator/log/drive_log.csv"  
  try:
    os.remove(logfile_nm)
  except OSError:
    pass
  logfile = open(logfile_nm, "w")
  logfile.write('time(s), tgt_heading(rad), tgt_range(m), steer_ang(rad), turn_radius(m), speed(m/s)\n')
  
  start_tm = rospy.get_time()
  
  rospy.Subscriber('/agv/steer_rpt', Floats, steer_rpt_callback)
  rospy.Subscriber('/agv/tgt_heading_rpt', Floats, tgt_callback)
  rospy.Subscriber('agv/speed', Float32, speed_callback)
  rospy.on_shutdown(shutdown_hook)
  rospy.spin()
