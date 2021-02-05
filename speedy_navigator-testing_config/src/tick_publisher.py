#!/usr/bin/python

import rospy
from std_msgs.msg import Bool
import RPi.GPIO as GPIO


if __name__ == '__main__':
  rospy.init_node('tick_publisher')
  pub = rospy.Publisher('/agv/tick', Bool, queue_size=10)

  try:
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)	# for channel A

    state = GPIO.input(27)

    while not rospy.is_shutdown():
      tick = GPIO.input(27)
      if tick != state:
        pub.publish(True)
        state = tick        
  finally:
    GPIO.cleanup()
