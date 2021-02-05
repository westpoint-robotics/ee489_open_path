#!/usr/bin/python

import rospy
import math
import numpy as np
from speedy_navigator.msg import Floats
from rospy.numpy_msg import numpy_msg
import vehicles

### Function that adjusts the perspective to a point a distance forward ###
### Takes 2 dimensional array of angles and ranges as input ###
def tf_adjust(ranges, adj_dist):
  old_ang, old_rng = ranges[:,0], ranges[:,1]
  ## Calculate new angles ##
  adj_ang = np.arctan2(old_rng*np.sin(old_ang), old_rng*np.cos(old_ang)-adj_dist)
  ## Calculate new ranges ##
  adj_rng = (old_rng*np.cos(old_ang)-adj_dist) / np.cos(adj_ang)
  ## Combine into one and sort ##
  adjusted = np.stack((adj_ang,adj_rng),axis=1)
  return adjusted[np.argsort(adjusted[:,0])]

def scan_callback(scan): 
  ranges_adjusted.data = tf_adjust(scan.data.reshape((-1,2),order='C'), laser2axle).flatten(order='C')
  pub.publish(ranges_adjusted)


if __name__ == '__main__':

  ranges_adjusted = Floats()	#data type to be published
  agv = vehicles.Traxxas()
  laser2axle = agv.laser2axle
  laser

  rospy.init_node('pov_transformer')
  rospy.Subscriber('/scan_filtered', numpy_msg(Floats), scan_callback)
  pub = rospy.Publisher('/agv/scan_transformed', numpy_msg(Floats), queue_size = 1)

  rospy.spin()
