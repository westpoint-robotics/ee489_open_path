def open_window_client(self, data):
    rospy.wait_for_service('open_window')
    try:
      open_window = rospy.ServiceProxy('open_window', PathData)
      output = open_window(data)
      return output.result
    except rospy.ServiceException:
      pass
    return 0
    
def pred_ranges_client(self, data):
    rospy.wait_for_service('pred_ranges')
    try:
      pred_ranges = rospy.ServiceProxy('pred_ranges', PathData)
      output = pred_ranges(data)
      return output.result
    except rospy.ServiceException:
      return 0
      
### Error adjustment from previous iteration (quasi-derivative controller) ###  
  def turn_adj(self, dtm):
    try:
      turn_radius = self.wheelbase/math.sin(self.steer_ang)  # positive counterclkwise
      if self.steer_ang > 0:
        turn_radius += self.half_width
      else:
        turn_radius -= self.half_width
      #self.publish_status("turn radius: {}".format(turn_radius))
      arc_ang = self.speed*dtm/turn_radius  # positive counterclkwise
      #self.publish_status("arc ang: {}".format(arc_ang))
      delta_y = turn_radius*math.sin(arc_ang) # positive up
      delta_x = turn_radius*(1 - math.cos(arc_ang)) # positive left
      #self.publish_status("delta y: {},  delta x: {}".format(delta_y,delta_x))
      #self.publish_status("aim dist: {}".format(self.aim_dist))
    except ZeroDivisionError:
      arc_ang = 0
      delta_y = self.speed*dtm
      delta_x = 0
    comp_ang = self.aim_steer - self.steer_ang
    #self.publish_status("comp ang: {}".format(comp_ang))
    try:
      new_comp_ang = math.atan((self.aim_dist*math.sin(comp_ang)-delta_x)\
       /(self.aim_dist*math.cos(comp_ang)-delta_y))
    except ZeroDivisionError:
      new_comp_ang = 0
    #self.publish_status("new aim ang: {}".format(new_aim_ang))
    turn_effect = arc_ang - new_comp_ang + comp_ang # vehicle itself
    #self.publish_status("turn effect: {}".format(turn_effect))
    return turn_effect
    
  ### Proportional integral controller ###  
  def pi_control(self, err, err_sum):
    return self.k_p*err + self.k_i*err_sum
    
  ### Entrapment check ###
  def check_entrapment(self, pred_ranges, window):
    ## window = [R_idx, R_ang, R_dist, L_idx, L_ang, L_dist]
    if pred_ranges and window:
      orig_window = window[:]
      try:
        window[1] = pred_ranges[2*int(window[0])]
        window[4] = pred_ranges[2*int(window[3])]
      except IndexError:
        pass
      r = self.sharpest_turn_radius
      #try:
      if window[4] < 0:	# If window is to the right
        ## find index for +90 deg ##
        bound_idx = 0
        try:
          while pred_ranges[2*bound_idx] < 0:
            bound_idx += 1
        except IndexError:
          bound_idx -= 1
        bound_idx -= 1
        for idx in range(int(window[3]), bound_idx):
          idx_ang = pred_ranges[2*idx]
          arc_ang = math.acos((r-self.half_width)*math.cos(idx_ang)/r)+idx_ang
          if pred_ranges[2*idx+1] < r*math.sin(arc_ang)/math.cos(idx_ang):
            self.publish_status("orig window: {}".format(orig_window))
            self.publish_status("window: {} to {}".format(window[1],window[4]))
            self.publish_status("idx ang: {}".format(idx_ang))
            self.publish_status("idx dist: {},  req dist: {}".format(pred_ranges[2*idx+1], r*math.sin(arc_ang)/math.cos(idx_ang)))
            if idx_ang > math.pi/2:
              print "idx_ang: {},  next ang: {}".format(idx_ang, pred_ranges[2*(idx-1)])
            return True
      elif window[1] > 0:	# If window is to the left
        ## find index for -90 deg ##
        bound_idx = 0
        try:
          while pred_ranges[2*bound_idx] < -math.pi/2:
            bound_idx += 1
        except IndexError:
          bound_idx -= 1
        for idx in range(int(window[0]), bound_idx, -1):
          idx_ang = pred_ranges[2*idx]
          arc_ang = math.acos((r-self.half_width)*math.cos(idx_ang)/r)+idx_ang
          if pred_ranges[2*idx+1] < r*math.sin(arc_ang)/math.cos(idx_ang):
            self.publish_status("orig window: {}".format(orig_window))
            self.publish_status("window: {} to {}".format(window[1],window[4]))
            self.publish_status("idx ang: {}".format(idx_ang))
            self.publish_status("idx dist: {},  req dist: {}".format(pred_ranges[2*idx+1], r*math.sin(arc_ang)/math.cos(idx_ang)))
            if idx_ang < -math.pi/2:
              print "idx_ang: {},  prev ang: {}".format(idx_ang, pred_ranges[2*(idx+1)])
            return True
      #except IndexError:
        #return True
    return False        

#self.publish_status("steer_ang: {},  aim_steer: {}".format(self.steer_ang, self.aim_steer))
turn_effect = self.turn_adj(new_tm-old_tm)
err = self.aim_steer - self.steer_ang
err_sum += err
#self.publish_status("err: {},  err_sum: {}".format(err,err_sum))
self.steer_ang += self.pi_control(err-turn_effect, err_sum)
#self.publish_status("new steer_ang: {}\n".format(self.steer_ang))
self.set_servos('steer')
## Publish current state parameters ##
params.data = [self.aim_steer, self.aim_dist, self.steer_ang, self.speed]
self.publish_params(params)
## get window and predicted range data ##
window = self.open_window_client(params.data)
try:
  pred_ranges = self.pred_ranges_client(params.data)
except rospy.exceptions.ROSInterruptException:
  pass
## Check for entrapment ##
try:
  entrapment = self.check_entrapment(pred_ranges, list(window))
except TypeError:
  self.publish_status("window error")
  continue
## retune steering
if entrapment:
  self.steer_ang = self.aim_steer
  self.throttle *= 0.85
  self.set_servos()
  self.publish_status("possible entrapment")
  
window = self.open_window_client(params.data)
try:
  pred_ranges = self.pred_ranges_client(params.data)
except rospy.exceptions.ROSInterruptException:
  pass
  
## Check for entrapment ##
entrapment = self.check_entrapment(pred_ranges, list(window))
if entrapment:
  self.steer_ang = self.aim_steer
  self.set_servos('steer')
  self.publish_status("possible entrapment")
  
  
def publish_window(data):
  window = data.data
  sphere_r, sphere_l = Marker(), Marker()
  idx = -2
  for sphere in sphere_r, sphere_l:
    idx += 3
    ang, dist = window[idx:idx+2]
    size = dist * 0.04
    sphere.header.frame_id = "shaft"
    sphere.ns = "open_window"
    sphere.id = idx
    sphere.type = 2
    sphere.action = 0
    sphere.scale.x = size
    sphere.scale.y = size
    sphere.scale.z = size
    sphere.pose.position.x = dist*math.cos(ang)
    sphere.pose.position.y = dist*math.sin(ang)
    sphere.pose.position.z = 0.0
    sphere.pose.orientation.x = 0.0
    sphere.pose.orientation.y = 0.0
    sphere.pose.orientation.z = 0.0
    sphere.pose.orientation.w = 1.0
    sphere.color.a = 1.0
    sphere.color.r = 1.0
    sphere.color.g = 0.0
    sphere.color.b = 1.0
    sphere.lifetime.secs = trax.loop_tm    
    window_pub.publish(sphere)
    
ang = start_ang - step_ang
filtered = []	# list for the filtered ranges
  try:
    for idx in range(int(round(len(nanless)/3.0))):	# average groups of 3
      i = 6*idx		#sets of 3 X 2 fields (angle and range) per set = 6
      ang1, rng1, ang2, rng2, ang3, rng3 = nanless[i], nanless[i+1], nanless[i+2], nanless[i+3], nanless[i+4], nanless[i+5]
      """
      av_rng = (rng1 + rng2 + rng3)/3.0
      try:
        inv_err1 = 1.0/abs(rng1-av_rng)
        inv_err2 = 1.0/abs(rng2-av_rng)
        inv_err3 = 1.0/abs(rng3-av_rng)
        av_ang = (inv_err1*ang1+inv_err2*ang2+inv_err3*ang3)/(inv_err1+inv_err2+inv_err3) # angle
      except ZeroDivisionError:
        if rng1 == av_rng:
          av_ang = ang1
        elif rng2 == av_rng:
          av_ang = ang2
        else:
          av_ang = ang3
      """
      min_rng = min(rng1, rng2, rng3)
      if min_rng == rng1:
        min_ang = ang1
      elif min_rng == rng2:
        min_ang = ang2
      else:
        min_ang = ang3
      filtered += [min_ang, min_rng]
      #filtered += [av_ang, av_rng]	# add average angle and range
  except IndexError:
    pass
  return filtered

def tf_adjust(ranges, adj_dist):
  ## Adjust the angle and range ##
  adjusted = []	# array for adjusted data
  for idx in range(len(ranges)/2):
    i = 2*idx	#data is represented in angle and range pairs
    old_ang = ranges[i] + trax.laser_tower_rot
    old_rng = ranges[i+1]
    adj_ang = math.atan(old_rng*math.sin(old_ang)/(old_rng*math.cos(old_ang)-adj_dist))
    if old_rng*math.cos(old_ang) < adj_dist:	# account for wrap
      if old_ang > 0:
        adj_ang += math.pi
      else:
        adj_ang -= math.pi
    adj_rng = (old_rng*math.cos(old_ang)-adj_dist) / math.cos(adj_ang)
    ## ensure the pair is inserted in ascending order by angle ##
    m = idx
    while m > 0 and adj_ang < adjusted[2*(m - 1)]:	
      m -= 1
    adjusted.insert(2*m, adj_ang)
    adjusted.insert(2*m+1, adj_rng)
  return adjusted
  
  open_ranges = []
  beam_ang = -max_steer_ang
  beam_idx = 0
  ## iterate from -24 to 24 deg
  while beam_ang < max_steer_ang:	    
    while ranges[2*beam_idx] < beam_ang:
      beam_idx += 1
    ## set right search index
    R_idx = beam_idx
    while (beam_ang - ranges[2*R_idx] < max_search_ang) and (R_idx > 0):
      R_idx -= 1
    ## set left search index
    L_idx = beam_idx
    try:
      while (ranges[2*L_idx] - beam_ang) < max_search_ang:
        L_idx += 1
    except IndexError:
      L_idx -= 1    
    min_dist_to_obst = 5.6
    for idx in range(R_idx+1, L_idx):
      i = idx*2
      off_ang = abs(beam_ang-ranges[i])
      req_dist = dist_lookup[int(off_ang//inc_ang)]
      if ranges[i+1] < req_dist:
        dist_to_obst = ranges[i+1]*math.cos(off_ang)
        if dist_to_obst < min_dist_to_obst:
          min_dist_to_obst = dist_to_obst
    # append obstruction data to return variable 
    open_ranges += [beam_ang, min_dist_to_obst]
    #update beam angle
    beam_ang += inc_ang
  return open_ranges
  
"""try:
      fwd_rng = self.max_sensor_rng
      best_idx = 0
      best_ang = data[0]
      best_dist = data[1]
      farthest_dist = 0
      farthest_val = 0
      val = 0
      for idx in range(1, len(data)/2):
        i = 2*idx
        idx_ang = data[i]
        idx_dist = data[i+1]
        if -self.scan_inc_ang < idx_ang < self.scan_inc_ang:	# extract forward range
          fwd_rng = idx_dist
        if idx_dist>5 and not round(math.degrees(idx_ang)):
          idx_val = 100
        else:
          idx_val = idx_dist/(1+(abs(idx_ang-old_ang)/(8*self.max_steer_ang)))
          #idx_val = idx_dist     
        if idx_val > val:
          val = idx_val
          best_idx = idx
          best_ang = idx_ang
          best_dist = idx_dist
        if idx_dist > farthest_dist:
          farthest_dist = idx_dist
          farthest_val = idx_val
    except IndexError:
      best_ang = old_ang
      best_dist = old_dist
      val = 0
    #self.publish_status("val: {},  ang: {}".format(val,best_ang))
    if farthest_dist > best_dist*1.5:
      print "best_dist: {},  best val: {},  far dist: {},  far val: {}".format(best_dist,val,farthest_dist,farthest_val)"""
      
### Function to calculate the average open distance ###
  def get_av_rng(self, paths):
    rng_sum = 0
    num_rngs = len(paths)/2
    for idx in range(num_rngs):
      rng_sum += paths[2*idx+1]
    return rng_sum/num_rngs
    
    
print '\nleft out'
  lo_scan = scan[scan[:,0]>-search_start]
  left_out_cond = (lo_scan[:,1]**2)+lo_scan[:,1]*(k1_left_out*np.sin(lo_scan[:,0]-steer_ang_out)+k2_left_out*np.cos(lo_scan[:,0]-steer_ang_out))+k3_left_out < 0
  if np.any(left_out_cond):
    #print np.argmax(left_out_cond)
    left_out_p = lo_scan[np.argmax(left_out_cond), 0]
    #print left_out_p
    left_out_limit = convert_to_center(left_out_p, rad_out, steer_ang_out, m_out, n_out, k1_left_out, k2_left_out, k3_left_out, 1)
  else:
    left_out_limit = max_scan_ang
  print '\nleft in'
  li_scan = scan_f[(scan_f[:,0]<search_start) & (scan_f[:,0]>search_in_end)]
  left_in_cond = (li_scan[:,1]**2)+li_scan[:,1]*(k1_left_in*np.sin(li_scan[:,0]-steer_ang_in)+k2_left_in*np.cos(li_scan[:,0]-steer_ang_in))+k3_left_in > 0
  if np.any(left_in_cond):
    left_in_p = li_scan[np.argmax(left_in_cond), 0]
    left_in_limit = convert_to_center(left_in_p, rad_in, steer_ang_in, m_in, n_in, k1_left_in, k2_left_in, k3_left_in, -1)
  else:
    left_in_limit = max_scan_ang
  left_limit = min(left_out_limit, left_in_limit)
  #print left_out_limit, left_in_limit, left_limit
  print '\nright out'
  ro_scan = scan_f[scan_f[:,0]<search_start]
  right_out_cond = (ro_scan[:,1]**2)+ro_scan[:,1]*(k1_right_out*np.sin(ro_scan[:,0]+steer_ang_out)+k2_right_out*np.cos(ro_scan[:,0]+steer_ang_out))+k3_right_out < 0
  if np.any(right_out_cond):
    right_out_p = ro_scan[np.argmax(right_out_cond), 0]
    right_out_limit = convert_to_center(right_out_p, -rad_out, -steer_ang_out, -m_out, n_out, k1_right_out, k2_right_out, k3_right_out, 1)
  else:
    right_out_limit = -max_scan_ang
  print '\nright in'
  ri_scan = scan[(scan[:,0]>-search_start) & (scan[:,0]<-search_in_end)]
  right_in_cond = (ri_scan[:,1]**2)+ri_scan[:,1]*(k1_right_in*np.sin(ri_scan[:,0]+steer_ang_in)+k2_right_in*np.cos(ri_scan[:,0]+steer_ang_in))+k3_right_in > 0
  if np.any(right_in_cond):
    right_in_p = ri_scan[np.argmax(right_in_cond), 0]
    right_in_limit = convert_to_center(right_in_p, -rad_in, -steer_ang_in, -m_in, n_in, k1_right_in, k2_right_in, k3_right_in, -1)
  else:
    right_in_limit = -max_scan_ang
  right_limit = max(right_out_limit, right_in_limit)
  
### function to convert the heading for a corner trajectory to the heading for the corresponding axle center trajectory ###
### if s=1, outside curve. if s=-1, inside curve ###
def convert_to_center(path_ang, rad, steer_ang, m, n, k1, k2, k3, s):
  #print path_ang, steer_ang, k1, k2, k3
  b = k1*sin(path_ang-steer_ang) + k2*cos(path_ang-steer_ang)
  c = k3
  B = asin((h*cos(path_ang-steer_ang)-n)/rad) # B = arc angle
  delta_x = m*cos(B) - n*sin(B)
  delta_y = -m*sin(B) - n*sin(B)
  center_path_ang = atan2(h*sin(path_ang)+delta_x, h*cos(path_ang)+delta_y)
  return center_path_ang
  
m_out = m_p*cos(steer_ang_out) + n_p*sin(steer_ang_out)
n_out = -m_p*sin(steer_ang_out) + n_p*cos(steer_ang_out)
m_in = -m_p*cos(steer_ang_out) + n_p*sin(steer_ang_out)
n_in = m_p*sin(steer_ang_out) + n_p*cos(steer_ang_out)

k1_left_out, k2_left_out, k3_left_out = 2*(m_out-rad_out), -2*n_out, (m_out**2)+(n_out**2)-(2*m_out*rad_out)
k1_left_in, k2_left_in, k3_left_in = 2*(m_in-rad_in), -2*n_in, (m_in**2)+(n_in**2)-(2*m_in*rad_in)
k1_right_out, k2_right_out, k3_right_out = 2*(-m_out+rad_out), -2*n_out, (-m_out**2)+(n_out**2)-(2*-m_out*-rad_out)
k1_right_in, k2_right_in, k3_right_in = 2*(-m_in+rad_in), -2*n_in, (-m_in**2)+(n_in**2)-(2*-m_in*-rad_in)

l_obs_n = -l_obs_m_p*np.sin(l_obs_alpha) + n_p*np.cos(l_obs_alpha)

## map PWM values to turn radii and steering angles ##
self.radius_pwm_pts = {-0.74: 395, -0.84: 380, -0.97: 370, -1.24: 355, -1.83: 335, -2.53: 300, -4.54: 298, 2.28: 295, 1.78: 265, 1.30: 250, 1.02: 240, 0.79: 230, 0.71: 210}
self.ang_pwm_pts = self.generate_ang_val_pts(self.radius_pwm_pts, self.wheelbase)
self.ref_angs = sorted(self.ang_pwm_map.keys())    
self.max_steer_ang = min(ref_angs[-1], abs(ref_angs[0]))

### return dictionary mapping of steering angles to values from input dictionary of turn radii to values ###
def generate_ang_val_pts(self, rad_val_pts, wheelbase):
  ang_val_pts = dict()
  for rad in rad_val_pts.keys():
    try:
      ang = math.asin(wheelbase/rad)
    except ZeroDivisionError:
      ang = 0
    ang_val_pts[ang] = rad_val_pts[rad]
  return ang_val_pts
  
### return dictionary mapping of steering angles to values from input dictionary of turn radii to values ###
def generate_ang_val_pts(self, rad_val_pts, wheelbase):
  ang_val_pts = dict()
  for rad in rad_val_pts.keys():
    try:
      ang = math.asin(wheelbase/rad)
    except ZeroDivisionError:
      ang = 0
    ang_val_pts[ang] = rad_val_pts[rad]
  return ang_val_pts
  
# Function that reduces the range data by 'factor' into one measurement
# per approx 1.05 degrees and replaces 'nan' values with the max scan range.
# Returns array of angles and ranges. Middle angles and min ranges per bucket are used.
def filt(ranges, range_min, range_max, start_ang, step_ang, factor):
  filtered = np.zeros((ranges.size//factor, 2), dtype=np.float32)
  ang = start_ang + step_ang*(fact-1)/2
  rng, prev1, prev2 = None, None, None
  it = np.nditer(ranges, flags=['c_index'], op_flags=[['readonly']])
  with it:
    while not it.finished:
      prev2 = prev1
      prev1 = rng
      rng = it[0]
      idx = it.index
      if math.isnan(rng) or math.isinf(rng):
        rng = range_max
      elif rng < range_min:
        if not prev1 == None:
          rng = prev1
        else:
          rng = range_min
      if not (idx+1)%fact:
        filtered[(idx+1-factor)/factor] = [ang, np.amin([rng, prev1, prev2])]
        ang += step_ang*fact
      it.iternext()
  return filtered
  
## iterate through array of steerable angles ##
open_rng, req_dist = None, None
outer_it = np.nditer([headings, open_rng], flags=['c_index'], op_flags=[['readonly'],['writeonly','allocate','no_broadcast']])
with outer_it:
  while not outer_it.finished:
    steer_idx = outer_it.index
    search_arr = points[np.absolute(points[:,0]-outer_it[0])<=ang_to_corner]
    off_idx = np.floor_divide(np.absolute(search_arr[:,0]-outer_it[0]),inc_ang).astype(np.intp)      
    req_dist = req_dist_lookup[off_idx]
    dist_to_obst = search_arr[search_arr[:,1]<req_dist,1] - axle2front
    try:
      outer_it[1] = np.amin(dist_to_obst)
    except ValueError:  #if array size of zero given
      outer_it[1] = max_scan_rng
    outer_it.iternext()
return np.stack((outer_it.operands[0],outer_it.operands[1]), axis=1)    


## if destination to the left ##      
if dest_heading > 0:
  ## determine distance of destination point from center of rotation ##
  dest_rad_diff_sqr = (dest_rng**2) - 2*self.max_turn_radius*dest_rng*sin(dest_heading-self.max_steer_ang)
  dest_rad = sqrt(dest_rad_diff_sqr+self.max_turn_radius**2)
  if dest_rad > self.max_turn_radius:
    theta = 0.5*self.left_turn_limit + self.max_steer_ang
    h = 2*self.max_turn_radius*sin(theta-self.max_steer_ang)
    dirac = atan2(dest_rng*sin(dest_heading)-h*sin(theta), dest_rng*cos(dest_heading)-h*cos(theta))
    if dirac < theta - pi:
      dirac += 2*pi
    if self.left_turn_limit+self.max_tgt_ang-0.38 > dirac:       
      return data[-1,0], data[-1,1], 0
## if destination to the right ##
else:
  dest_rad_diff_sqr = (dest_rng**2) + 2*self.max_turn_radius*dest_rng*sin(dest_heading+self.max_steer_ang)
  dest_rad = sqrt(dest_rad_diff_sqr+self.max_turn_radius**2)
  if dest_rad > self.max_turn_radius:
    theta = 0.5*self.right_turn_limit - self.max_steer_ang
    h = -2*self.max_turn_radius*sin(theta+self.max_steer_ang)
    dirac = atan2(dest_rng*sin(dest_heading)-h*sin(theta), dest_rng*cos(dest_heading)-h*cos(theta))
    if dirac > theta + pi:
      dirac -= 2*pi
    if self.right_turn_limit-self.max_tgt_ang+0.38 < dirac:
      return data[0,0], data[0,1], 0
      
def throttle_ramp(speed_err, curr_throttle, max_throttle, pwm_inc, speed_inc):
  if abs(speed_err) < speed_inc:
    inc = 1
  else:
    inc = pwm_inc
  if speed_err > 0:
    out = curr_throttle + inc
  else:
    out = curr_throttle - inc
  if out > max_throttle:
    return max_throttle
  elif out < 0:
    return 0
  else:
    return out
    
dirac_prime = atan2(1-cos(arc_ang), sin(arc_ang))
dirac = dirac_prime + steer_ang
h = turn_radius*sin(arc_ang)/cos(dirac_prime)
delta_x, delta_y = h*sin(dirac), h*cos(dirac)

def update_pose(delta_x, delta_y, delta_ang, steer_angle, arc_ang, arc_dist):
  ## Runge-Kutta method of vehicle motion estimation for the LIDAR sensor location ##
  ## The following are the translations and rotations for the LIDAR from scan start to end ##
  delta_x += arc_dist*sin(delta_ang + (arc_ang/2) + steer_ang_lidar)
  delta_y += arc_dist*cos(delta_ang + (arc_ang/2) + steer_ang_lidar)
  delta_ang += arc_ang
  return delta_x, delta_y, delta_ang
  
scan_angs = np.where(np.isnan(scan[:,0])|np.isinf(scan[:,0]), math.pi, scan[:,0])
