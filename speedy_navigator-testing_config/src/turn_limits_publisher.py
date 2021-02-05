#!/usr/bin/python

import rospy
import numpy as np
from math import sin, asin, cos, acos, atan2, sqrt, pi
from speedy_navigator.msg import Floats
from rospy.numpy_msg import numpy_msg
import vehicles

def scan_callback(msg):
  global limits
  scan = msg.data.reshape(-1,2,order='C')
  scan_f = scan[::-1]   # inverted for analyzing the right hand curves  
  ## left curve: ##
  l_scan = scan[scan[:,0]>-search_start]
  l_rad_diff_sqr = (l_scan[:,1]**2)-2*rad_center*l_scan[:,1]*np.sin(l_scan[:,0]-steer_ang_center)
  l_rad = np.sqrt(l_rad_diff_sqr+rad_center**2)
  l_obs_cond = (l_rad<rad_out) & (l_rad>rad_in)
  if np.any(l_obs_cond):
    l_obs, l_obs_rad = l_scan[l_obs_cond], l_rad[l_obs_cond]    
    l_obs_alpha = np.arcsin((wheelbase+n_p)/l_obs_rad)    
    l_obs_m_p = l_obs_rad*np.sin(np.arccos((wheelbase+n_p)/l_obs_rad)) - cor2axle_x
    l_obs_m = l_obs_m_p*np.cos(l_obs_alpha) + n_p*np.sin(l_obs_alpha)
    l_wrap_cond = l_obs[:,0] < l_obs_alpha + pi/2
    l_obs_B1 = np.arccos(1 - (l_obs_m[l_wrap_cond]+l_obs[l_wrap_cond,1]*np.sin(l_obs[l_wrap_cond,0]-l_obs_alpha[l_wrap_cond]))/l_obs_rad[l_wrap_cond])
    l_obs_B2 = 2*pi - np.arccos(1 - (l_obs_m[~l_wrap_cond]+l_obs[~l_wrap_cond,1]*np.sin(l_obs[~l_wrap_cond,0]-l_obs_alpha[~l_wrap_cond]))/l_obs_rad[~l_wrap_cond])
    l_obs_B = np.concatenate((l_obs_B1, l_obs_B2))
    l_obs_delta_x = l_obs_m_p*np.cos(l_obs_B) - n_p*np.sin(l_obs_B)
    l_obs_delta_y = -l_obs_m_p*np.sin(l_obs_B) - n_p*np.cos(l_obs_B)
    l_lims = np.arctan2(l_obs[:,1]*np.sin(l_obs[:,0])+l_obs_delta_x, l_obs[:,1]*np.cos(l_obs[:,0])+l_obs_delta_y)
    l_lims = l_lims[l_lims>0]
    try:
      left_limit_idx = np.argmin(l_lims)
      left_limit_ang = l_lims[left_limit_idx]
      left_limit_arc_ang = l_obs_B[left_limit_idx]
    except ValueError:
      left_limit_ang = max_scan_ang
      left_limit_arc_ang = max_arc_ang
  else:
    left_limit_ang = max_scan_ang
    left_limit_arc_ang = max_arc_ang
  if left_limit_ang > max_scan_ang:
    left_limit_ang = max_scan_ang
    left_limit_arc_ang = max_arc_ang 
  ## right curve: ##
  r_scan = scan_f[scan_f[:,0]<search_start]
  r_rad_diff_sqr = (r_scan[:,1]**2)+2*rad_center*r_scan[:,1]*np.sin(r_scan[:,0]+steer_ang_center)
  r_rad = -np.sqrt(r_rad_diff_sqr+rad_center**2)
  r_obs_cond = (r_rad>-rad_out) & (r_rad<-rad_in)
  if np.any(r_obs_cond):
    r_obs, r_obs_rad = r_scan[r_obs_cond], r_rad[r_obs_cond]
    r_obs_alpha = np.arcsin((wheelbase+n_p)/r_obs_rad)
    r_obs_m_p = r_obs_rad*np.sin(np.arccos((wheelbase+n_p)/r_obs_rad)) + cor2axle_x
    r_obs_m = r_obs_m_p*np.cos(r_obs_alpha) + n_p*np.sin(r_obs_alpha)
    r_wrap_cond = r_obs[:,0] > r_obs_alpha - pi/2
    r_obs_B1 = -np.arccos(1 - (r_obs_m[r_wrap_cond]+r_obs[r_wrap_cond,1]*np.sin(r_obs[r_wrap_cond,0]-r_obs_alpha[r_wrap_cond]))/r_obs_rad[r_wrap_cond])
    r_obs_B2 = -2*pi + np.arccos(1 - (r_obs_m[~r_wrap_cond]+r_obs[~r_wrap_cond,1]*np.sin(r_obs[~r_wrap_cond,0]-r_obs_alpha[~r_wrap_cond]))/r_obs_rad[~r_wrap_cond])
    r_obs_B = np.concatenate((r_obs_B1, r_obs_B2))     
    r_obs_delta_x = r_obs_m_p*np.cos(r_obs_B) - n_p*np.sin(r_obs_B)
    r_obs_delta_y = -r_obs_m_p*np.sin(r_obs_B) - n_p*np.cos(r_obs_B)
    r_lims = np.arctan2(r_obs[:,1]*np.sin(r_obs[:,0])+r_obs_delta_x, r_obs[:,1]*np.cos(r_obs[:,0])+r_obs_delta_y)
    r_lims = r_lims[r_lims<0]    
    try:
      right_limit_idx = np.argmax(r_lims)
      right_limit_ang = r_lims[right_limit_idx]
      right_limit_arc_ang = r_obs_B[right_limit_idx]
    except ValueError:
      right_limit_ang = -max_scan_ang
      right_limit_arc_ang = -max_arc_ang
  else:
    right_limit_ang = -max_scan_ang
    right_limit_arc_ang = -max_arc_ang
  if right_limit_ang < -max_scan_ang:
    right_limit_ang = -max_scan_ang
    right_limit_arc_ang = -max_arc_ang
  limits.data = [right_limit_ang, right_limit_arc_ang, left_limit_ang, left_limit_arc_ang]
  pub.publish(limits)


if __name__ == '__main__':
  limits = Floats()
  agv = vehicles.Traxxas()  
  wheelbase, steer_ang_center, rad_center = agv.wheelbase, agv.max_steer_ang, agv.max_turn_radius
  max_scan_ang, scan_res = agv.max_scan_ang, agv.scan_res
  
  np.warnings.filterwarnings('ignore')
  
  ## Initialize constants: ##
  theta_c = acos(wheelbase/rad_center)
  cor2axle_x, cor2axle_y = rad_center*sin(theta_c), rad_center*cos(theta_c)
  max_arc_ang = 2*(max_scan_ang - steer_ang_center)
    
  m_p, n_p = agv.m_p, agv.n_p  # offset distances of outside front corner from axle center
  rad_out = sqrt((rad_center*cos(steer_ang_center)+m_p)**2 + (rad_center*sin(steer_ang_center)+n_p)**2)       #front outside corner
  steer_ang_out = atan2(rad_center*sin(steer_ang_center)+n_p, rad_center*cos(steer_ang_center)+m_p)
  rad_in = sqrt((rad_center*cos(steer_ang_center)-m_p)**2 + (rad_center*sin(steer_ang_center)-wheelbase)**2)  #back inside corner
  #rad_in = sqrt((rad_center*cos(steer_ang_center)-m_p)**2 + (rad_center*sin(steer_ang_center)+n_p)**2)  #front inside corner
  steer_ang_in = atan2(rad_center*sin(steer_ang_center)+n_p, rad_center*cos(steer_ang_center)-m_p)
    
  search_start = atan2(m_p, n_p)  

  rospy.init_node('turn_limits_publisher')
  pub = rospy.Publisher('/agv/turn_limits', Floats, queue_size=1)
  rospy.Subscriber('/agv/scan_filtered', numpy_msg(Floats), scan_callback)
  
  #tpub = rospy.Publisher('/agv/troubleshoot', numpy_msg(Floats))
  
  rospy.spin()
