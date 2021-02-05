#!/usr/bin/python

import rospy, math
import numpy as np
from sensor_msgs.msg import LaserScan
from speedy_navigator.msg import Floats
from rospy.numpy_msg import numpy_msg
import vehicles
  
# Function that reduces the range data by 'factor' into one measurement
# per approx 1.05 degrees and replaces 'nan' values with the max scan range.
# Returns array of angles and ranges. Average angles and min ranges per bucket are used.  
def get_reduced(data, factor):
  filt_num = np.size(data, axis=0)//factor
  filt_idx = np.arange(filt_num)*factor
  #filtered = (data[filt_idx] + data[filt_idx+1] + data[filt_idx+2])/factor
  ## out of every three points, take the point with minimum range ##
  filtered = data[np.argmin(np.stack((data[filt_idx,1],data[filt_idx+1,1],data[filt_idx+2,1]),axis=1),axis=1)+filt_idx]  
  return filtered
 
### replace 'nan' and 'inf' values ###
def scan_callback_reduce(msg):
  global ranges_filtered
  scan = msg.data.reshape(-1,2,order='C')
  scan = np.delete(scan, np.nonzero(np.isnan(scan[:,0])|np.isinf(scan[:,0])), axis=0)
  scan_rngs = np.where(np.isnan(scan[:,1])|np.isinf(scan[:,1]), max_scan_rng, scan[:,1]) 
  ranges_filtered.data = np.reshape(get_reduced(np.stack((scan[:,0],scan_rngs),axis=1), filt_factor), -1, order='C')    
  filter_pub.publish(ranges_filtered)
  
### replace 'nan' and 'inf' values ###
def scan_callback_no_reduction(msg):
  global ranges_filtered
  scan = msg.data.reshape(-1,2,order='C')
  scan = np.delete(scan, np.nonzero(np.isnan(scan[:,0])|np.isinf(scan[:,0])), axis=0)
  scan_rngs = np.where(np.isnan(scan[:,1])|np.isinf(scan[:,1]), max_scan_rng, scan[:,1]) 
  ranges_filtered.data = np.stack((scan[:,0],scan_rngs),axis=1).flatten(order='C')
  filter_pub.publish(ranges_filtered)


if __name__ == '__main__':
  agv = vehicles.Traxxas()
  max_scan_rng = agv.max_scan_rng 
  ranges_filtered = Floats()	# data type to be published
  
  rospy.init_node('scan_filter')
  filter_pub = rospy.Publisher('agv/scan_filtered', numpy_msg(Floats), queue_size=1)
  
  if agv.do_reduce:  
#    filt_factor = int((2*agv.half_width/3)//(agv.max_scan_rng*agv.scan_res)) # reduce data quantity by this much
    filt_factor = 3
    rospy.Subscriber('agv/scan_transformed', numpy_msg(Floats), scan_callback_reduce)
  else:
    rospy.Subscriber('agv/scan_transformed', numpy_msg(Floats), scan_callback_no_reduction)  

  rospy.spin()
