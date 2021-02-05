#!/usr/bin/python

import rospy, sys, os
sys.path.append(os.path.dirname(__file__)+'/..')
import vehicles
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState

def joint_state_callback(msg):
  global left_wheel_ref, right_wheel_ref, overflow
  left_wheel, right_wheel = msg.position[0], msg.position[5]
  ang_disp = ((left_wheel-left_wheel_ref)+(right_wheel-right_wheel_ref))/2 + overflow
  if ang_disp > rad_per_tick:
    tick_pub.publish(True)
    overflow = ang_disp - rad_per_tick
    left_wheel_ref, right_wheel_ref = left_wheel, right_wheel  
  elif -ang_disp > rad_per_tick:
    tick_pub.publish(False)
    overflow = ang_disp + rad_per_tick
    left_wheel_ref, right_wheel_ref = left_wheel, right_wheel 


if __name__ == '__main__':
  agv = vehicles.Traxxas()
  rad_per_tick = agv.meters_per_tick/agv.tire_radius
  left_wheel_ref, right_wheel_ref, overflow = 0.0, 0.0, 0.0

  rospy.init_node('tick_publisher')
  tick_pub = rospy.Publisher('/agv/tick', Bool, queue_size=10)
  rospy.Subscriber('ackermann_vehicle/joint_states', JointState, joint_state_callback) 
  
  rospy.spin()
