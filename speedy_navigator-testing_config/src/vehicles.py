import math
import numpy as np

class Traxxas:

  def __init__(self):
    self.init_stampede(True)
    #self.init_emaxx(True)
    self.init_lidar(self.lidar_type)
    self.width = 2*self.half_width
    self.ang_to_corner = math.atan(self.half_width/self.axle2front)
    self.steer_res = 0.01
    self.max_turn_radius = self.wheelbase/math.sin(self.max_steer_ang)
    self.max_tgt_ang = 1.7*self.max_steer_ang
#    self.max_tgt_ang = self.max_steer_ang + self.steer_res
    self.m_p, self.n_p = self.half_width, self.axle2front # offset distances of outside front corner from axle center for collision checking
    self.control_loop_rate = 333    # rate at which steering corrections are made    
    
  
  def init_stampede(self, simulation=False):
    self.half_width = 0.165 + 0.05	# half of vehicle width plus cushion
    self.track_div_2 = 0.135
    self.axle2front = 0.0625    
    self.tire_radius = 0.125/2
    self.tire_width = 0.06
    self.wheelbase = 0.270
    self.laser2axle = 0.08	# distance from laser to front axle in centimeters
    self.do_reduce = True
#    self.av_steer_response = 2*self.max_steer_ang/0.5  # rad/s
    self.av_steer_response = 2.0
    if simulation:
      self.simulation = True
      self.lidar_type = 1
      self.destination_frame_id = 'base'
      self.laser_rotation = 0.0
      self.do_correct_distortion = False
      self.do_correct_delay = False
      self.approx_process_delay = 0.008       
      self.use_speed = True   #whether to calculate distance traveled directly from ticks or indirectly via speed.
      self.max_steer_ang = 0.3
      self.max_speed = 5			# full throttle vehicle speed in m/s
      self.meters_per_tick = 0.005    
    else:
      self.simulation = False
      self.lidar_type = 0
      self.destination_frame_id = 'front_axle_center'
      self.laser_rotation = math.radians(2.0)
      self.do_correct_distortion = True
      self.do_correct_delay = True
      self.approx_process_delay = 0.03          
      self.use_speed = False   #whether to calculate distance traveled directly from ticks or indirectly via speed. 
      ## PWM ##
      self.pwm_radius_pts = {210:0.71, 230:0.79, 240:1.02, 250:1.30, 265:1.78, 295:2.28, 298:-4.54, 300:-2.53, 335:-1.83, 355:-1.24, 370:-0.97, 380:-0.84, 395:-0.74}
      self.pwm_ang_pts = self.generate_pwm_ang_pts(self.pwm_radius_pts, self.wheelbase)
      self.ref_pwms = sorted(self.pwm_ang_pts.keys())
      self.max_steer_ang = min(abs(self.pwm_ang_pts[self.ref_pwms[0]]), abs(self.pwm_ang_pts[self.ref_pwms[-1]]))      
      ## PWM values for throttle ##
      self.throttle_neutral = 315	# PWM value for sitting still    
      self.max_throttle = 105		# max forward speed: throttle = 315+105=420
      self.max_speed = 1.8			# full throttle vehicle speed in m/s
      self.pwm_speed_ratio = self.max_throttle/self.max_speed	# ratio of speed to pwm change
      self.meters_per_tick = 0.001486  # for encoder    
    
    
  def init_emaxx(self, simulation=True):
    self.half_width = 0.2085 + 0.03
    self.track_div_2 = 0.1575
    self.axle2front = 0.073
    self.tire_radius = 0.14605/2
    self.tire_width = 0.070
    self.wheelbase = 0.335
    self.laser_rotation = 0.0
    self.laser2axle = 0.08	# distance from laser to front axle in centimeters
    #self.max_inside_steer_ang = 0.41
    self.max_steer_ang = 0.3
    self.steer_response_kp = 8.73
    self.av_steer_response = 1.09
    self.meters_per_tick = 0.1
    self.max_speed = 3.0
    self.lidar_type = 1
    self.do_correct_distortion = False
    self.do_correct_delay = False
    self.do_reduce = True
    self.use_speed = True
    
    
  def init_lidar(self, kind=0):
    # hokuyo_urg04lx01 ##
    if kind == 0:
      self.scan_rate = 10
#      self.scan_rate = 50
      self.max_scan_ang = 2.1
      self.scan_res = 0.00613592332229
      self.max_scan_rng = 5.5
    ## gazebo ray sensor ##
    elif kind == 1:
      self.scan_rate = 10
      self.max_scan_ang = 2.0944
      num_samples = 683
      self.scan_res = 2*self.max_scan_ang/num_samples
      self.max_scan_rng = 15
    self.scan_prd = float(1.0/self.scan_rate)
    
    
  def get_cushion(self, max_steer_ang):
    max_turn_radius = self.wheelbase/math.sin(max_steer_ang)
    m = self.m_p*math.cos(max_steer_ang) + self.n_p*math.sin(max_steer_ang)
    n = -self.m_p*math.sin(max_steer_ang) + self.n_p*math.cos(max_steer_ang)
    k1, k2, k3 = 2*(m-max_turn_radius), -2*n, (m**2)+(n**2)-(2*m*max_turn_radius)
    b = k2*math.cos(max_steer_ang) - k1*math.sin(max_steer_ang)
    c = k3
    h = (-b+math.sqrt((b**2)-4*c))/2
    B = math.asin((h*math.cos(max_steer_ang)-n)/max_turn_radius)
    return B, h
        
    
  ### generate mapping of steering angles to PWM values ###
  def generate_ang_pwm_map(self):
    pwms = np.arange(self.ref_pwms[0], self.ref_pwms[-1], dtype=int)
    steer_angs = np.empty((len(pwms)))
    idx = 0
    for pwm in pwms:
      ang = self.pwm_to_ang(pwm)
      steer_angs[idx] = ang
      idx += 1
    return np.stack((steer_angs, pwms), axis=1)[np.argsort(steer_angs)]
    
    
  ### return dictionary mapping of PWM values to steering angles from input dictionary of PWM values to turn radii ###
  def generate_pwm_ang_pts(self, pwm_rad_pts, wheelbase):
    pwm_ang_pts = dict()
    for pwm, rad in pwm_rad_pts.items():
      try:
        ang = math.asin(wheelbase/rad)
      except ZeroDivisionError:
        ang = 0
      pwm_ang_pts[pwm] = ang
    return pwm_ang_pts
  
  ### map PWM value to steer angle ###  
  def pwm_to_ang(self, pwm):
    if pwm < self.ref_pwms[0]:
      return self.pwm_ang_pts[ref_pwms[0]]
    elif pwm > self.ref_pwms[-1]:
      return self.pwm_ang_pts[self.ref_pwms[-1]]
    i = 0
    try:
      while self.ref_pwms[i] <= pwm:
        i += 1
    except IndexError:
      i -= 1
    ratio = (self.pwm_ang_pts[self.ref_pwms[i]]-self.pwm_ang_pts[self.ref_pwms[i-1]])/ \
     (self.ref_pwms[i]-self.ref_pwms[i-1])
    return ratio*(pwm-self.ref_pwms[i-1]) + self.pwm_ang_pts[self.ref_pwms[i-1]]
    
  ### Translate outside steering angles to effective angle at different axle location given displacment to outside of curve ###
  def ackermann_shift(self, orig_angs, disp):
    if not type(orig_angs) is list:
      radius = self.wheelbase/math.sin(orig_angs)
      return math.atan2(radius*math.sin(orig_angs), radius*math.cos(orig_angs)+disp)
    center_angs = []
    for ang in orig_angs:
      radius = self.wheelbase/math.sin(orig_angs)
      new_ang = math.atan2(radius*math.sin(ang), radius*math.cos(ang)+disp)
      center_angs.append(new_ang)
    return center_angs
    
  def generate_traverse_center_trajectory(self, speed_inc, steer_ang_inc):
    tm_inc = steer_ang_inc/self.av_steer_response      
    max_steer_ang = self.max_steer_ang
    num_angs = int(round(2*max_steer_ang/steer_ang_inc)) + 1
    steer_angs = np.linspace(-max_steer_ang, max_steer_ang, num=num_angs)
    num_speeds = int(self.max_speed//speed_inc)+1
    speeds = np.linspace(0, self.max_speed, num=num_speeds)
    steer_ang_v, speed_v = np.meshgrid(steer_angs, speeds)   
    heading_v = speed_v*(math.cos(steer_angs[0])-np.cos(steer_ang_v))/(self.av_steer_response*self.wheelbase)
    orientation = heading_v + steer_ang_v
    delta_x = speed_v*tm_inc*np.cos(orientation)
    delta_y = speed_v*tm_inc*np.sin(orientation)
    x = np.array([[np.sum(delta_x[i,:j]) for j in range(num_angs)] for i in range(num_speeds)])
    y = np.array([[np.sum(delta_y[i,:j]) for j in range(num_angs)] for i in range(num_speeds)])
    return steer_angs, x, y, heading_v
    
  def generate_traverse_trajectories(self, speed_inc, steer_ang_inc):
    r2l_steer_angs, center_x, r2l_center_y, r2l_headings = self.generate_traverse_center_trajectory(speed_inc, steer_ang_inc)
    l2r_steer_angs = -r2l_steer_angs
    l2r_headings = -r2l_headings
    r2l_center = np.stack((center_x, r2l_center_y), axis=2)
    l2r_center = np.stack((center_x, -r2l_center_y), axis=2)
    cos_heading, sin_heading = np.cos(r2l_headings), np.sin(r2l_headings)
    right_x = center_x + sin_heading*self.m_p + cos_heading*self.n_p
    r2l_right_y = r2l_center_y - cos_heading*self.m_p + sin_heading*self.n_p
    r2l_right = np.stack((right_x, r2l_right_y), axis=2)
    num_top = int(2*self.m_p//0.01)
    left_x = center_x - sin_heading*self.m_p + cos_heading*self.n_p 
    r2l_left_y = r2l_center_y + cos_heading*self.m_p + sin_heading*self.n_p
    r2l_left = np.stack((left_x, r2l_left_y), axis=2)
    top_x = np.meshgrid(np.ones(num_top), left_x[:,-1])[1]
    r2l_top_y = r2l_center_y[:,-1].reshape((np.size(r2l_center_y,axis=0),1)) + np.linspace(-self.m_p, self.m_p, num_top)
    r2l_top = np.stack((top_x, r2l_top_y), axis=2)
    r2l_box = np.concatenate((r2l_right, r2l_top, r2l_left[:,::-1]), axis=1)    
    l2r_right = np.stack((r2l_left[:,:,0], -r2l_left[:,:,1]), axis=2)
    l2r_top = np.stack((r2l_top[:,:,0], -r2l_top[:,:,1]), axis=2)
    l2r_left = np.stack((r2l_right[:,:,0], -r2l_right[:,:,1]), axis=2)
    l2r_box = np.concatenate((l2r_right, l2r_top[:,::-1], l2r_left[:,::-1]), axis=1)    
    num_angs = np.size(r2l_steer_angs)
    corners = np.array([num_angs, num_angs+num_top]).astype(np.intp)
    return r2l_steer_angs, r2l_headings, r2l_center, r2l_box, l2r_headings, l2r_center, l2r_box, corners
    
  def ackermann_linkage(self):
    track = 2*self.track_div_2
    max_turn_radius = self.wheelbase/math.sin(self.max_steer_ang)
    inside_max_steer_ang = math.atan(self.wheelbase/((self.wheelbase/math.tan(self.max_steer_ang))-self.track_div_2))
    outside_max_steer_ang = math.atan(self.wheelbase/((self.wheelbase/math.tan(self.max_steer_ang))+self.track_div_2))
    linkage_ang = math.atan(self.wheelbase/self.track_div_2)
    A = linkage_ang - inside_max_steer_ang
    B = linkage_ang + outside_max_steer_ang
    quad_a = 1 - (1+math.cos(A+B))/(2*math.cos(linkage_ang)**2)
    quad_b = track*((1+math.cos(A+B))/(math.cos(linkage_ang)**2) - (math.cos(A)+math.cos(B))/math.cos(linkage_ang))
    quad_c = -track**2*(1 - (math.cos(A)+math.cos(B))/math.cos(linkage_ang) + (1+math.cos(A+B))/(2*math.cos(linkage_ang)**2))
    ackermann_bar_length = (-quad_b+math.sqrt(quad_b**2 -4*quad_a*quad_c))/(2*quad_a)
    side_bar_length = (track-ackermann_bar_length)/(2*math.cos(linkage_ang))    
    print '\n', track
    print inside_max_steer_ang
    print outside_max_steer_ang
    print linkage_ang
    print A
    print B
    print math.cos(A) + math.cos(B), math.cos(A+B)
    print quad_a
    print quad_b
    print quad_c
    print ackermann_bar_length
    print side_bar_length
    print '\n', math.sqrt((track-side_bar_length*(math.cos(A)+math.cos(B)))**2 + (side_bar_length*(math.sin(A)-math.sin(B)))**2)
    print track-2*side_bar_length*math.cos(linkage_ang)
