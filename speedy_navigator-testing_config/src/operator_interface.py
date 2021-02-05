#!/usr/bin/python

### Key press code adapted from https://www.darkcoding.net/software/non-blocking-console-io-is-not-possible/ ###

import sys, select, termios, tty, os, time
import rospy
from std_msgs.msg import Int8, String

def status_callback(msg):
  print "\r\t--{}".format(msg.data)
    
def delay(dtm):
  ctm = rospy.get_time()
  while rospy.get_time()-ctm < dtm:
    pass
  return
  
def cleanup():
    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    
    
if __name__ == '__main__':
  rospy.init_node('operator_interface')
  key_pub = rospy.Publisher('/agv/keypress', Int8, queue_size=1)
  rospy.Subscriber('/agv/status_msg', String, status_callback)
  print "\nSelect a destination on RVIZ. Press ENTER or SPACE to start and stop. Press 's' to shutdown remote robot operation."
  
  key_delay = 0.1
  fd = sys.stdin.fileno()
  old_settings = termios.tcgetattr(fd)
  rospy.on_shutdown(cleanup)
  tty.setcbreak(sys.stdin.fileno())
  
#  while not rospy.is_shutdown():
#    try:
#      if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
#        char = sys.stdin.read(1)
#        if char in '\n ':
#          key_pub.publish(1)
#          delay(key_delay)
#        elif char == '=':
#          key_pub.publish(2)
#          delay(key_delay)
#        elif char == '-':
#          key_pub.publish(3)
#          delay(key_delay)
#        elif char == '1':
#          key_pub.publish(11)
#          delay(key_delay)
#        elif char == '2':
#          key_pub.publish(12)
#          delay(key_delay)
#        elif char == '3':
#          key_pub.publish(13)
#          delay(key_delay)
#        elif char in 'sS':
#          key_pub.publish(0)
#          delay(key_delay)
#    except:
#      pass

  rospy.spin()
