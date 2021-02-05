#!/usr/bin/python

import rospy
from math import sqrt, sin, cos, atan2, pi
import numpy as np
from rospy.numpy_msg import numpy_msg
from speedy_navigator.msg import Floats, WayPoint
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Int8, Header
from tf.transformations import quaternion_from_euler
import vehicles

def setup_markers():
  global right_curve, left_curve, dest_marker
  ref_pose = Pose()
  ref_pose.position.x, ref_pose.position.y, ref_pose.position.z = 0.0, 0.0, 0.0
  ref_quat = quaternion_from_euler(0.0, 0.0, 0.0)
  ref_pose.orientation.x = ref_quat[0]
  ref_pose.orientation.y = ref_quat[1]
  ref_pose.orientation.z = ref_quat[2]
  ref_pose.orientation.w = ref_quat[3]
  
  right_curve.header.frame_id = "front_axle_center"
  right_curve.ns = "curves"
  right_curve.id = 0
  right_curve.type = 4
  right_curve.action = 0
  right_curve.pose = ref_pose
  right_curve.scale.x = 0.01
  right_curve.color.a = 1.0
  right_curve.color.r = 1.0
  right_curve.color.g = 1.0
  right_curve.color.b = 0.0
  right_curve.lifetime.secs = 0.1
  
  left_curve.header.frame_id = "front_axle_center"
  left_curve.ns = "curves"
  left_curve.id = 1
  left_curve.type = 4
  left_curve.action = 0
  left_curve.pose = ref_pose
  left_curve.scale.x = 0.01
  left_curve.color.a = 1.0
  left_curve.color.r = 1.0
  left_curve.color.g = 1.0
  left_curve.color.b = 0.0
  left_curve.lifetime.secs = 0.1
  
  ref_pose = Pose()
  ref_pose.position.x, ref_pose.position.y, ref_pose.position.z = 0.0, 0.0, 0.0
  ref_quat = quaternion_from_euler(0.0, 0.0, 0.0)
  ref_pose.orientation.x = ref_quat[0]
  ref_pose.orientation.y = ref_quat[1]
  ref_pose.orientation.z = ref_quat[2]
  ref_pose.orientation.w = ref_quat[3]

  dest_marker.header.frame_id = "front_axle_center"
  dest_marker.ns = "waypoint"
  dest_marker.id = 0
  dest_marker.type = 2
  dest_marker.pose = ref_pose
  dest_marker.color.a = 1.0
  dest_marker.color.r = 1.0
  dest_marker.color.g = 0.5
  dest_marker.color.b = 0.0
  dest_marker.lifetime.secs = 0.1
  return
  
def generic_arrow():
  ref_pose = Pose()
  ref_pose.position.x, ref_pose.position.y, ref_pose.position.z = 0.0, 0.0, 0.0
  generic_arrow = Marker()
  generic_arrow.header.frame_id = "front_axle_center"
  generic_arrow.ns = "open_paths"
  generic_arrow.type = 0
  generic_arrow.action = 0
  generic_arrow.pose = ref_pose
  generic_arrow.scale.x = 1
  generic_arrow.lifetime.secs = 0.1
  return generic_arrow
  
def del_arrow():
  del_arrow = Marker()
  del_arrow.header.frame_id = "front_axle_center"
  del_arrow.ns = "open_paths"
  del_arrow.type = 0
  del_arrow.action = 2
  return del_arrow

def generate_curve(rad, start_ang, max_ang, ang_res):
  curve_angs = np.linspace(start_ang, max_ang, (max_ang-start_ang)//ang_res)
  curve_rngs = 2*rad*np.sin(curve_angs-start_ang)
  return np.stack((curve_angs, curve_rngs, curve_rngs*np.cos(curve_angs), curve_rngs*np.sin(curve_angs)), axis=1)
  
def generate_corner_curve(center_curve, rad_center, max_steer_ang, corner_rad, m_p, n_p):
  wrap_cond = center_curve[:,0] < max_steer_ang + pi/2
  B1 = np.arccos(1 - center_curve[wrap_cond,1]*np.sin(center_curve[wrap_cond,0]-max_steer_ang)/rad_center)
  B2 = 2*pi - np.arccos(1 - center_curve[~wrap_cond,1]*np.sin(center_curve[~wrap_cond,0]-max_steer_ang)/rad_center)
  B = np.concatenate((B1, B2))
  delta_x = m_p*np.cos(B) - n_p*np.sin(B)
  delta_y = -m_p*np.sin(B) - n_p*np.cos(B)
  x = center_curve[:,1]*np.cos(center_curve[:,0]) - delta_y
  y = center_curve[:,1]*np.sin(center_curve[:,0]) - delta_x
  curve_angs = np.arctan2(y, x)
  curve_rngs = np.sqrt((x**2) + (y**2))
  return np.stack((curve_angs, curve_rngs, x, y), axis=1)

def destination_callback(msg):
  global dest_marker
  heading, rng = msg.heading, msg.range
  if heading and rng:    
    dest_marker.action = 0
    dest_marker.header.stamp.secs = rospy.get_time()
    dest_marker.pose.position.x = rng*cos(heading)
    dest_marker.pose.position.y = rng*sin(heading)  
    dest_marker.scale.x = 0.35
    dest_marker.scale.y = 0.35
    dest_marker.scale.z = 0.35
    dest_marker.color.a = 1.0
  else:
    dest_marker.action = 2
  dest_marker_pub.publish(dest_marker)    
  
def draw_curves(msg):
  global curve_array, right_curve, left_curve
  r_lim, l_lim = msg.data[0], msg.data[2]
  curve_array.markers = []
  right_curve.points = []
  for point in curve_coords[curve_coords[:,0]<=-r_lim]:
    curve_pt = Point()
    curve_pt.x = point[2]
    curve_pt.y = -point[3]
    curve_pt.z = 0
    right_curve.points.append(curve_pt)
  curve_array.markers.append(right_curve)
  left_curve.points = []
  for point in curve_coords[curve_coords[:,0]<=l_lim]:
    curve_pt = Point()
    curve_pt.x = point[2]
    curve_pt.y = point[3]
    curve_pt.z = 0
    left_curve.points.append(curve_pt)
  curve_array.markers.append(left_curve)
  
  ref_pose = Pose()
  ref_pose.position.x, ref_pose.position.y, ref_pose.position.z = 0.0, 0.0, 0.0
  ref_quat = quaternion_from_euler(0.0, 0.0, 0.0)
  ref_pose.orientation.x = ref_quat[0]
  ref_pose.orientation.y = ref_quat[1]
  ref_pose.orientation.z = ref_quat[2]
  ref_pose.orientation.w = ref_quat[3]
  right_out = Marker()
  right_out.header.frame_id = "front_axle_center"
  right_out.ns = "curves"
  right_out.id = 2
  right_out.type = 4
  right_out.action = 0
  right_out.pose = ref_pose
  right_out.scale.x = 0.01
  right_out.color.a = 1.0
  right_out.color.r = 0.0
  right_out.color.g = 0.0
  right_out.color.b = 1.0
  right_out.lifetime.secs = 0.1
  right_out.points = []
  curve_right_out = curve_coords_out[curve_coords[:,0]<-r_lim]
  for point in curve_right_out[curve_right_out[:,0]>0.0]:
    curve_pt = Point()
    curve_pt.x = point[2]
    curve_pt.y = -point[3]
    curve_pt.z = 0
    right_out.points.append(curve_pt)
  curve_array.markers.append(right_out)
  right_in = Marker()
  right_in.header.frame_id = "front_axle_center"
  right_in.ns = "curves"
  right_in.id = 3
  right_in.type = 4
  right_in.action = 0
  right_in.pose = ref_pose
  right_in.scale.x = 0.01
  right_in.color.a = 1.0
  right_in.color.r = 0.0
  right_in.color.g = 0.0
  right_in.color.b = 1.0
  right_in.lifetime.secs = 0.1
  right_in.points = []
  curve_right_in = curve_coords_in[curve_coords[:,0]<-r_lim]
  for point in curve_right_in:
    curve_pt = Point()
    curve_pt.x = point[2]
    curve_pt.y = -point[3]
    curve_pt.z = 0
    right_in.points.append(curve_pt)
  curve_array.markers.append(right_in)
  left_out = Marker()
  left_out.header.frame_id = "front_axle_center"
  left_out.ns = "curves"
  left_out.id = 4
  left_out.type = 4
  left_out.action = 0
  left_out.pose = ref_pose
  left_out.scale.x = 0.01
  left_out.color.a = 1.0
  left_out.color.r = 0.0
  left_out.color.g = 0.0
  left_out.color.b = 1.0
  left_out.lifetime.secs = 0.1
  left_out.points = []
  curve_left_out = curve_coords_out[curve_coords[:,0]<l_lim]
  for point in curve_left_out[curve_left_out[:,0]>0.0]:
    curve_pt = Point()
    curve_pt.x = point[2]
    curve_pt.y = point[3]
    curve_pt.z = 0
    left_out.points.append(curve_pt)
  curve_array.markers.append(left_out)
  left_in = Marker()
  left_in.header.frame_id = "front_axle_center"
  left_in.ns = "curves"
  left_in.id = 5
  left_in.type = 4
  left_in.action = 0
  left_in.pose = ref_pose
  left_in.scale.x = 0.01
  left_in.color.a = 1.0
  left_in.color.r = 0.0
  left_in.color.g = 0.0
  left_in.color.b = 1.0
  left_in.lifetime.secs = 0.1
  left_in.points = []
  curve_left_in = curve_coords_in[curve_coords[:,0]<l_lim]
  for point in curve_left_in:
    curve_pt = Point()
    curve_pt.x = point[2]
    curve_pt.y = point[3]
    curve_pt.z = 0
    left_in.points.append(curve_pt)
  curve_array.markers.append(left_in)  
  curve_markers_pub.publish(curve_array)
  
def steer_rpt_callback(msg): # [steer_ang, turn_radius]
  steer_ang = msg.data[0]
  ## draw steering arrow ##
  try:
    steer_dist = vectors[np.searchsorted(vectors[:,0], steer_ang), 1]
  except (IndexError, TypeError):
    steer_dist = 0
  arrow = generic_arrow()
  arrow.id = 1001
  arrow.action = 0
  quat = quaternion_from_euler(0, 0, steer_ang)
  arrow.pose.orientation.x = quat[0]
  arrow.pose.orientation.y = quat[1]
  arrow.pose.orientation.z = quat[2]
  arrow.pose.orientation.w = quat[3]
  arrow.scale.x = steer_dist
  arrow.scale.y = 0.02
  arrow.scale.z = 0.02
  arrow.color.a = 1.0
  arrow.color.r = 0.0
  arrow.color.g = 0.0
  arrow.color.b = 1.0
  arrow.lifetime.secs = 0.1
  steer_marker_pub.publish(arrow)
  
def tgt_callback(msg):
  tgt_heading, tgt_rng = msg.data
  ## draw tgt arrow ##
  arrow = generic_arrow()
  arrow.id = 1000
  quat = quaternion_from_euler(0, 0, tgt_heading)
  arrow.pose.orientation.x = quat[0]
  arrow.pose.orientation.y = quat[1]
  arrow.pose.orientation.z = quat[2]
  arrow.pose.orientation.w = quat[3]
  arrow.scale.x = tgt_rng
  arrow.scale.y = 0.02
  arrow.scale.z = 0.02
  arrow.color.a = 1.0
  arrow.color.r = 1.0
  arrow.color.g = 0.0
  arrow.color.b = 0.0
  arrow.lifetime.secs = 0.1
  aim_marker_pub.publish(arrow)
  

def publish_paths(paths):
  global vectors
  arrow_array.markers = []
  vectors = paths.data.reshape(-1,2,order='C')   
  num_vectors = len(vectors)
  steerable_cond = (vectors[:,0]>=-max_steer_ang) & (vectors[:,0]<=max_steer_ang)
  steerable = vectors[steerable_cond]
  num_steerable = len(steerable)
  fringe = vectors[~steerable_cond]
  steer_ang_found = False
  for idx in range(num_steerable):
    heading = steerable[idx,0]
    arrow = generic_arrow()
    arrow.id = idx
    quat = quaternion_from_euler(0, 0, heading)
    arrow.pose.orientation.x = quat[0]
    arrow.pose.orientation.y = quat[1]
    arrow.pose.orientation.z = quat[2]
    arrow.pose.orientation.w = quat[3]
    arrow.scale.x = steerable[idx,1]
    arrow.scale.y = 0.005
    arrow.scale.z = 0.005
    arrow.color.a = 1.0
    arrow.color.r = 0.0
    arrow.color.g = 1.0
    arrow.color.b = 0.0
    arrow_array.markers.append(arrow)
  for idx in range(num_vectors-num_steerable):
    arrow = generic_arrow()
    arrow.id = idx + num_steerable
    quat = quaternion_from_euler(0, 0, fringe[idx,0])
    arrow.pose.orientation.x = quat[0]
    arrow.pose.orientation.y = quat[1]
    arrow.pose.orientation.z = quat[2]
    arrow.pose.orientation.w = quat[3]
    arrow.scale.x = fringe[idx,1]
    arrow.scale.y = 0.005
    arrow.scale.z = 0.005
    arrow.color.a = 1.0
    arrow.color.r = 0.0
    arrow.color.g = 1.0
    arrow.color.b = 1.0
    arrow_array.markers.append(arrow)
  for idx in range(num_vectors, arrow_max_idx):
    arrow = del_arrow()
    arrow.id = idx
    arrow_array.markers.append(arrow)      
  paths_pub.publish(arrow_array)  
  
def publish_pointcloud(msg):
  points = msg.data.reshape(-1,2,order='C')
  x_arr, y_arr = points[:,1]*np.cos(points[:,0]), points[:,1]*np.sin(points[:,0])
  z_arr = np.zeros((x_arr.size))
  struct_arr = np.array([tuple(pt) for pt in np.stack((x_arr,y_arr,z_arr),axis=1)], dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4')])
  pointcloud = pc2.create_cloud_xyz32(pc_header, struct_arr)
  pointcloud_pub.publish(pointcloud)
 

if __name__ == '__main__':
  agv = vehicles.Traxxas()
  rad_center = agv.max_turn_radius
  max_steer_ang = agv.max_steer_ang
  vectors = []
  
  m_p, n_p = agv.m_p, agv.n_p
  rad_out = sqrt((rad_center*cos(max_steer_ang)+m_p)**2 + (rad_center*sin(max_steer_ang)+n_p)**2)
  max_steer_out = atan2(rad_center*sin(max_steer_ang)+n_p, rad_center*cos(max_steer_ang)+m_p)
  rad_in = sqrt((rad_center*cos(max_steer_ang)-m_p)**2 + (rad_center*sin(max_steer_ang)+n_p)**2)
  max_steer_in = atan2(rad_center*sin(max_steer_ang)+n_p, rad_center*cos(max_steer_ang)-m_p)
    
  dest_marker = Marker()
  curve_array, right_curve, left_curve = MarkerArray(), Marker(), Marker() 
  curve_coords = generate_curve(rad_center, max_steer_ang, pi+max_steer_ang, agv.scan_res)
  
  curve_coords_out = generate_corner_curve(curve_coords, rad_center, max_steer_ang, rad_center, m_p, n_p)
  curve_coords_in = generate_corner_curve(curve_coords, rad_center, max_steer_ang, rad_center, -m_p, n_p)
  
  arrow_array = MarkerArray()
  arrow_max_idx = int(2*agv.max_tgt_ang//agv.steer_res)
  setup_markers()
  
  pc_header = Header()
  pc_header.frame_id = "front_axle_center"
  
  rospy.init_node('visualizer')
  paths_pub = rospy.Publisher('/agv/open_path_arrows', MarkerArray, queue_size=1)
  aim_marker_pub = rospy.Publisher('/agv/aim_marker', Marker, queue_size=1)
  steer_marker_pub = rospy.Publisher('/agv/steer_marker', Marker, queue_size=1)
  dest_marker_pub = rospy.Publisher('/agv/dest_marker', Marker, queue_size=1)
  curve_markers_pub = rospy.Publisher('/agv/curve_markers', MarkerArray, queue_size=1)  
  pointcloud_pub = rospy.Publisher('/agv/pointcloud', PointCloud2, queue_size=1)
  rospy.Subscriber('/agv/destination', WayPoint, destination_callback)
  if agv.do_correct_delay:
    rospy.Subscriber('/agv/open_paths_corrected', numpy_msg(Floats), publish_paths)
  else:
    rospy.Subscriber('/agv/open_paths', numpy_msg(Floats), publish_paths)    
  rospy.Subscriber('/agv/steer_rpt', Floats, steer_rpt_callback)
  rospy.Subscriber('/agv/tgt_heading_rpt', Floats, tgt_callback)
  rospy.Subscriber('/agv/turn_limits', Floats, draw_curves)
  rospy.Subscriber('/agv/scan_transformed', numpy_msg(Floats), publish_pointcloud)
  #rospy.Subscriber('/agv/troubleshoot', numpy_msg(Floats), publish_pointcloud)  
  print "\ndisplaying markers\n\r"
  rospy.spin()
