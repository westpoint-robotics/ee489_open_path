#!/usr/bin/python

import rospy
from std_msgs.msg import Bool, Float32
import vehicles

tick = 0
count = 0

def tick_callback(tick):
  global count
  if tick.data:
    count += 1
  else:
    count -= 1

if __name__ == '__main__':
  rospy.init_node('speed_publisher')

  agv = vehicles.Traxxas()  
  count, old_count = 0, 0
  oldtm = rospy.get_time()
  meters_per_tick = agv.meters_per_tick
  speed = 0
  
  rospy.Subscriber('/agv/tick', Bool, tick_callback)
  speed_pub = rospy.Publisher('/agv/speed', Float32, queue_size=1)  

  rate = rospy.Rate(10)   
  while not rospy.is_shutdown():
    new_count = count
    newtm = rospy.get_time()
    try:
      speed = meters_per_tick*(new_count - old_count)/(newtm - oldtm)
      old_count, oldtm = new_count, newtm
    except ZeroDivisionError:
      if speed:
        if rospy.get_time() - new_ticktm > 2*meters_per_tick/abs(speed):
          speed = 0
    speed_pub.publish(speed)

    rate.sleep()
