#!/usr/bin/python

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32

def scan_callback(data):
  global tm
  tm = rospy.get_time()
  
def paths_callback(data):
  global tm
  delay = rospy.get_time() - tm
  pub.publish(delay)

if __name__ == '__main__':
  tm = 0
  rospy.init_node('delay_publisher')
  rospy.Subscriber('/scan', LaserScan, scan_callback)
  rospy.Subscriber('/open_paths', Float32MultiArray, paths_callback)
  pub = rospy.Publisher('/delay', Float32, queue_size=10)  
  rospy.spin()
