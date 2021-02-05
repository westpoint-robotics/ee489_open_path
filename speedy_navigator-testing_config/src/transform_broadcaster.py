#!/usr/bin/python

import rospy
import tf2_ros
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler
import vehicles

if __name__ == '__main__':
  rospy.init_node('tf2_broadcaster')
  agv = vehicles.Traxxas()
  laser2axle, laser_rotation = agv.laser2axle, agv.laser_rotation
  
  broadcaster = tf2_ros.StaticTransformBroadcaster()
  static_transformStamped = geometry_msgs.msg.TransformStamped()

  static_transformStamped.header.stamp = rospy.Time.now()
  static_transformStamped.header.frame_id = "front_axle_center"
  static_transformStamped.child_frame_id = "laser"

  static_transformStamped.transform.translation.x = -laser2axle
  static_transformStamped.transform.translation.y = 0
  static_transformStamped.transform.translation.z = 0
  quat = quaternion_from_euler(0, 0, laser_rotation)
  static_transformStamped.transform.rotation.x = quat[0]
  static_transformStamped.transform.rotation.y = quat[1]
  static_transformStamped.transform.rotation.z = quat[2]
  static_transformStamped.transform.rotation.w = quat[3]

  broadcaster.sendTransform(static_transformStamped)
  rospy.spin()
