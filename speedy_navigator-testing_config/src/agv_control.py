#!/usr/bin/python

import rospy
from std_msgs.msg import Int8, Bool, Float32, String
from speedy_navigator.msg import Floats, WayPoint
from sensor_msgs.msg import LaserScan
from rospy.numpy_msg import numpy_msg
from math import pi, sqrt, cos, sin, tan, atan2
import numpy as np
import vehicles

class AGVDrive(vehicles.Traxxas):

  def __init__(self):
    vehicles.Traxxas.__init__(self)
    ## Configure parameters and state variables ##
    self.do_run = False
    self.stopped = False
    self.op_mode = 1   # 1=FreeFlow, 2=Navigation, 3=JoyDrive
    self.obstructed = True
    self.destination = WayPoint()
    self.destination.heading, self.destination.range = 0.0, 0.0  
    self.speed, self.tgt_speed = 0.0, 3.0
    self.max_turn_limit = 2*(self.max_scan_ang - self.max_steer_ang)
    self.left_turn_limit, self.right_turn_limit = self.max_turn_limit, -self.max_turn_limit
    self.cushion_B, self.cushion_h = self.get_cushion(self.max_steer_ang)
    self.tgt_rpt = Floats()
    self.tgt_heading, self.tgt_rng, self.dest_head_rng = 0, 0.1, 0
    self.steer_ang = self.tgt_heading	# set initial wheel orientation
    self.turn_radius = 0
    self.raw_scan_received_tm = rospy.get_time()
    self.scan_received = False
    self.aim_updated_tm = rospy.get_time()
    ## ROS Publisher/Subscriber Setup ##
    self.opmode_pub = rospy.Publisher('/agv/op_mode', Int8, queue_size=10)
    self.status_pub = rospy.Publisher('/agv/status_msg', String, queue_size=10)
    self.tgt_rpt_pub = rospy.Publisher('/agv/tgt_heading_rpt', Floats, queue_size=1)
    self.steer_cmd_pub = rospy.Publisher('/agv/steer_cmd', Float32, queue_size=1)
    self.speed_cmd_pub = rospy.Publisher('/agv/speed_cmd', Float32, queue_size=10)    
    rospy.Subscriber('/agv/speed', Float32, self.speed_callback)
    if not self.use_speed:
      self.tick_count = 0
      rospy.Subscriber('/agv/tick', Bool, self.tick_callback)    
    rospy.Subscriber('/agv/keypress', Int8, self.keypress_callback)
    rospy.Subscriber('/agv/destination', WayPoint, self.dest_callback)
    rospy.Subscriber('/agv/turn_limits', Floats, self.turn_limits_callback)
    if self.do_correct_delay:
      rospy.Subscriber('/agv/open_paths_corrected', numpy_msg(Floats), self.open_paths_callback)
    else:
      rospy.Subscriber('/agv/open_paths', numpy_msg(Floats), self.open_paths_callback)
    #rospy.Subscriber('/agv/scan_arrived', Bool, self.scan_signal_callback)
    ## final setup ##
    self.publish_status("target speed: {} m/s".format(self.tgt_speed))
    self.set_steer_ang(self.steer_ang)
    self.set_speed()
    self.publish_status("Ready: starting in Free Flow mode")
    
  ### Callback and Service Functions ###
  def scan_signal_callback(self, msg):
    self.raw_scan_received_tm = rospy.get_time()
  
  ### Operator Input ###
  def keypress_callback(self, msg):
    ## if in Nav or Free Flow mode ##
    if not self.op_mode == 3:
      ## start/stop ##
      if msg.data == 1:
        self.do_run = not self.do_run
        if self.do_run:
          if self.op_mode == 2 and not self.destination.range:
            self.do_run = False
            self.publish_status("select a destination to begin navigating")
          else:
            self.publish_status("running")
        else:
          self.publish_status("stopping")
    ## increase speed ##
    if msg.data == 2 and self.tgt_speed < self.max_speed:
      self.tgt_speed += 1
      if self.tgt_speed > self.max_speed:
        self.tgt_speed = self.max_speed
      if self.do_run:
        self.set_speed(self.tgt_speed)
      self.publish_status("target speed: {} m/s".format(self.tgt_speed))
    ## decrease speed ##
    elif msg.data == 3 and self.tgt_speed > 1:
      self.tgt_speed -= 1
      if self.tgt_speed < 1:
        self.tgt_speed = 1
      if self.do_run:
        self.set_speed(self.tgt_speed)
      self.publish_status("target speed: {} m/s".format(self.tgt_speed))
    ## select operation mode ##      
    elif msg.data == 11:
      if not self.op_mode == 1:
        self.op_mode = 1
        self.opmode_pub.publish(1)
        self.publish_status("entering Free Flow mode")
    elif msg.data == 12:
      if not self.op_mode == 2:
        self.op_mode = 2
        self.opmode_pub.publish(2)
        self.publish_status("entering Navigation mode")
    elif msg.data == 13:
      if not self.op_mode == 3:
        self.op_mode = 3
        self.opmode_pub.publish(3)
        self.do_run = False
        self.stopped = False
        self.publish_status("entering Joy Drive mode")
    ## kill AGV ##  
    elif msg.data == 0:
      rospy.signal_shutdown("robot killed by user")    
      sys.exit()
      
  def speed_callback(self, msg):
    self.speed = msg.data
    
  def tick_callback(self, tick):
    if tick.data:
      self.tick_count += 1
    else:
      self.tick_count -= 1
    
  ### publish status messages ###
  def publish_status(self, msg):
    try:
      self.status_pub.publish(msg)
    except (rospy.exceptions.ROSInterruptException, rospy.exceptions.ROSException):
      pass
    return
  
  ### publish steer command ###
  def set_steer_ang(self, steer_ang):
    if steer_ang < -self.max_steer_ang:
      steer_ang = -self.max_steer_ang
    elif steer_ang > self.max_steer_ang:
      steer_ang = self.max_steer_ang
    self.steer_ang = steer_ang
    try:
      self.steer_cmd_pub.publish(steer_ang)
    except (rospy.exceptions.ROSInterruptException, rospy.exceptions.ROSException):
      pass
    ## return turn radius ##
    try:
      return self.wheelbase/sin(steer_ang)
    except ZeroDivisionError:
      return 0
    
  ### publish speed command ###
  def set_speed(self, speed=0):
    try:
      self.speed_cmd_pub.publish(speed)
    except (rospy.exceptions.ROSInterruptException, rospy.exceptions.ROSException):
      pass
    return
  
  ### destination update ###
  def dest_callback(self, msg):
    self.destination = msg
    if self.destination.is_new:
      self.publish_status("destination selected")
      if self.op_mode == 1:
        self.op_mode = 2
        self.opmode_pub.publish(2)
        self.publish_status("entering Navigation mode")
    elif self.op_mode == 2 and self.do_run and self.destination.range < self.arrived_margin:
      self.publish_status("arrived!")
      self.do_run = False
      self.op_mode = 1
      self.opmode_pub.publish(1)
      self.publish_status("entering Free Flow mode")
        
  ### turn limits update ###
  def turn_limits_callback(self, msg):
    self.right_turn_limit = msg.data[1]
    self.left_turn_limit = msg.data[3]
      
  ### open paths update...also update aiming data ### 
  def open_paths_callback(self, paths):
    open_paths = paths.data.reshape(-1,2,order='C')    
    if np.any(open_paths):
      # if Navigation Mode #
      if self.op_mode == 2:
        self.tgt_heading, self.tgt_rng, self.dest_head_rng = \
         self.pick_heading_navmode(open_paths, self.destination.heading, self.destination.range, self.tgt_heading)
      # if Free Flow Mode #
      else:
        self.tgt_heading, self.tgt_rng = self.pick_heading_freeflow(open_paths, self.tgt_heading)
        self.dest_head_rng = 0
      self.scan_received = True  
      self.tgt_rpt.data = [self.tgt_heading, self.tgt_rng]
      try:
        self.tgt_rpt_pub.publish(self.tgt_rpt)
      except:
        pass
    elif self.do_run:
      self.do_run = False
      self.publish_status("stopping: nowhere to go!")
    self.aim_updated_tm = rospy.get_time()
    
  ### pick a heading for the vehicle based on distance open and angular proximity to destination heading ###
  def pick_heading_navmode(self, data, dest_heading, dest_rng, old_heading):
    if data[0,0] <= dest_heading <= data[-1,0]:
      dest_head_rng = data[np.searchsorted(data[:,0], dest_heading), 1]
      if dest_head_rng > dest_rng:
        if self.obstructed:
          self.obstructed = False
          self.publish_status("destination in view")
        return dest_heading, dest_rng, dest_head_rng 
      else:
        self.obstructed = True    
    ## if destination is outside of targetable open path range ##
    else:
      dest_head_rng = 1
      left_val = self.is_reachable(dest_heading, dest_rng, self.left_turn_limit, data[-1,0], 1)
      right_val = self.is_reachable(dest_heading, dest_rng, self.right_turn_limit, data[0,0], -1)
      if left_val > right_val:
        return data[-1,0], data[-1,1], 0
      elif right_val > left_val:
        return data[0,0], data[0,1], 0
    ## if not targeting destination directly ##
    old_head = tgt_heading
    if dest_head_rng > self.cushion_h:    
      tgt_heading, tgt_rng = data[np.argmax(data[:,1]/(1+(np.absolute(data[:,0]-dest_heading)/(2*self.max_tgt_ang))))]
    else:
      tgt_heading, tgt_rng = data[np.argmax(data[:,1])]      
    return tgt_heading, tgt_rng, dest_head_rng
    
  ### check if new heading is viable given wheel traverse rate limitation ###
  def viable_heading(self, new_heading, old_heading, traverse_rate, speed, ranges):
    if new_heading>old_heading:
      check = data[(data[:0]>old_heading)&(data[:,0]<new_heading)]
    elif new_heading>old_heading:
      check = data[(data[:0]<old_heading)&(data[:,0]>new_heading)]
    else:
      return True
    eff_steer_ang = (old_heading + old_heading)/2
    eff_turn_rad = self.wheelbase/sin(eff_steer_ang)
    arc_ang = speed*(new_heading-old_heading)/traverse_rate
    x = eff_turn_rad*(cos(eff_steer_ang) - cos(eff_steer_ang+arc_ang)) + self.half_width*cos(arc_ang)
    y = eff_turn_rad*-(sin(eff_steer_ang) - sin(eff_steer_ang+arc_ang)) + self.n_p*sin(arc_ang)
    ratio = y/(x-self.half_width*cos(arc_ang))
    test = (data[:,1]*cos(data[:,0]))/(data[:,1]*sin(data[:,0])) < ratio
    if np.any(test):
      return False
    else:
      return True
    
  ### determine if a point is likely to be able to be reached by turning the wheels all the way to one side ###
  ### direction and limit of turn are given by the 'turn_limit' and 'd' inputs ###
  ### if likely to be possible, return by how much ###
  def is_reachable(self, pt_heading, pt_rng, turn_limit, scan_limit, d=None):
    ## determine direction ##
    if not d:
      if turn_limit > 0:
        d = 1
      else:
        d = -1
    ## determine distance of destination point from center of rotation ##
    pt_rad_diff_sqr = (pt_rng**2) - 2*d*self.max_turn_radius*pt_rng*sin(pt_heading-d*self.max_steer_ang)
    pt_rad = sqrt(pt_rad_diff_sqr+self.max_turn_radius**2)
    if pt_rad + self.arrived_margin > self.max_turn_radius:
      theta = 0.5*turn_limit + d*self.max_steer_ang
      h = 2*d*self.max_turn_radius*sin(theta-d*self.max_steer_ang)
      dirac = atan2(pt_rng*sin(pt_heading)-h*sin(theta), pt_rng*cos(pt_heading)-h*cos(theta))
      if d*pt_heading < 0 or d*dirac < d*theta - pi:
        dirac += d*2*pi
      if pt_rad < self.max_turn_radius + self.arrived_margin:
        try:  
          d_p = abs((pt_rng*sin(pt_heading)-h*sin(theta))/sin(dirac))
        except ZeroDivisionError:
          d_p = abs((pt_rng*cos(pt_heading)-h*cos(theta))/cos(dirac))
        if d_p < self.arrived_margin:
          if d*pt_heading < 0:
            return 1
          else:
            return 2
        q = atan2(pt_rng*sin(pt_heading)-d*self.max_turn_radius*cos(self.max_steer_ang), pt_rng*cos(pt_heading)+self.max_turn_radius*sin(self.max_steer_ang))
        if d*q < self.max_steer_ang - (pi/2):
          q += d*2*pi
        if d*turn_limit > d*q - self.max_steer_ang + (pi/2):
          if d*pt_heading < 0:
            return 1
          else:
            return 2          
      margin = d*(turn_limit - dirac + scan_limit) - self.cushion_B
      if margin > 0:
        if d*pt_heading < 0:
          return 1
        else:
          return 2     
      elif d*turn_limit > self.max_turn_limit-0.01:
        if d*pt_heading < 0:
          return -1
        else:
          return 0    
    return -2
     
  ## if Free Flow mode with no destination ##
  def pick_heading_freeflow(self, data, old_heading):
    try:
      forward_rng = data[np.searchsorted(data[:,0], 0.0), 1]
    except IndexError:
      forward_rng = 0.0
    if forward_rng > 5:
      tgt_heading, tgt_rng = 0.0, forward_rng
    else:
      #tgt_heading, tgt_rng = data[np.argmax(data[:,1])]
      tgt_heading, tgt_rng = data[np.argmax(data[:,1]/(1+(np.absolute(data[:,0]-old_heading)/(8*self.max_steer_ang))))]
    return tgt_heading, tgt_rng
    
  ### Update aiming point based on odometry in between LIDAR updates ###  
  def update_tgt_point(self, old_ang, old_rng, steer_ang, turn_radius, dist):
    try:
      arc_ang = dist/turn_radius  # positive is counterclockwise
      delta_x = turn_radius*(cos(steer_ang)-cos(steer_ang+arc_ang))
      delta_y = turn_radius*(sin(steer_ang+arc_ang)-sin(steer_ang))
    except ZeroDivisionError:
      arc_ang = 0
      delta_x, delta_y = 0, dist
    new_x, new_y = old_rng*sin(old_ang)-delta_x, old_rng*cos(old_ang)-delta_y
    theta = atan2(new_x, new_y)
    new_ang = theta - arc_ang
    new_rng = sqrt(new_x**2 + new_y**2)   
    return new_ang, new_rng
    
  ### Main thread to run the robot ###  
  def run_robot(self):
    tgt_heading, tgt_rng, turn_radius = self.steer_ang, self.tgt_rng, 0
    if self.use_speed:
      steer_tm = rospy.get_time()
    else:
      steer_ticks = self.tick_count
    rate = rospy.Rate(self.control_loop_rate)
    while not rospy.is_shutdown():
      if self.do_run:
        #looptm = rospy.get_time()
        if self.stopped:
          self.set_speed(self.tgt_speed)
          self.stopped = False
        if self.scan_received:  # synchronize operation with reciept of LIDAR data
          tgt_heading, tgt_rng = self.tgt_heading, self.tgt_rng
          turn_radius = self.set_steer_ang(tgt_heading)
          if self.use_speed:
            steer_tm = rospy.get_time()
          else:
            steer_ticks = self.tick_count
          #print "\nprocess time:", rospy.get_time()-self.raw_scan_received_tm
          self.scan_received = False
          
          #print '\nLIDAR UPDATED'
          #print 'tgt:', tgt_heading, tgt_rng
          
        ## if break in scan data stream ##  
        elif rospy.get_time()-self.aim_updated_tm>1.0:
          self.do_run = False
          self.publish_status("stopping: break in scan input")
        ## between LIDAR updates: ##
        else:
          if self.use_speed:          
            new_tm = rospy.get_time()
            dist_traveled = self.speed * (new_tm-steer_tm)
            steer_tm = new_tm
          else:
            new_ticks = self.tick_count
            dist_traveled = self.meters_per_tick * (new_ticks-steer_ticks)
            steer_ticks = new_ticks
          if dist_traveled:
            tgt_heading, tgt_rng = self.update_tgt_point(tgt_heading, tgt_rng, tgt_heading, turn_radius, dist_traveled)
            turn_radius = self.set_steer_ang(tgt_heading)
          
          #if dist_traveled: 
            #print '\ndist:', dist_traveled
            #print 'tgt:', tgt_heading, tgt_rng
                      
        #print 'loop tm:', rospy.get_time() - looptm
        #print 'tgt heading:', tgt_heading, ' tgt_rng:', tgt_rng
      ## if not running or in Joy Drive mode ##  
      elif not self.op_mode == 3:
        if not self.stopped:
          self.set_speed()  # bring vehicle to a stop
          self.stopped = True
        self.aim_updated_tm = rospy.get_time()
        if self.speed:
          self.set_steer_ang(self.steer_ang)          
      rate.sleep()     
        
  ### Actions before shutdown ### 
  def shutdown_hook(self):
    self.set_speed()   # stop vehicle
    self.publish_status("shutting down")


if __name__ == '__main__':
  rospy.init_node('agv_control')
  agv = AGVDrive()
  rospy.on_shutdown(agv.shutdown_hook)
  agv.run_robot()   
