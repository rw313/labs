#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from rbx1_nav.transform_utils import quat_to_angle, normalize_angle
from sensor_msgs.msg import LaserScan
from math import pi, pow, atan2, sqrt, radians, copysign, degrees
import os
import math 

class Bug2():
    def __init__(self):
        rospy.init_node('bug2', anonymous=True) #tru anon means multiple listeners can run simultaneously 
        rospy.on_shutdown(self.shutdown)
        
        self.cmd_vel = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=1)        

	self.tf_listener = tf.TransformListener()
	rospy.sleep(2) #give tf some time to fill buffer 
	self.odom_frame = '/odom' 
	try:
	    self.tf_listener.waitForTransform(self.odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))
	    self.base_frame = '/base_footprint'
	except (tf.Exception, tf.ConnectivityException, tf.LookupException):
	    try:
		self.tf_listener.waitForTransform(self.odom_frame, '/base_link', rospy.Time(), rospy.Duration(1.0))
		self.base_frame = '/base_link'
	    except (tf.Exception, tf.ConnectivityException, tf.LookupException):
		rospy.loginfo("Cannot find transform between odom and base link or base footprint")
		rospy.signal_shutdown("tf Exception") 
	self.start = Point()
	(self.start, self.start_rot) = self.get_odom()
	self.pos = Point()
	self.update_odom() 
        self.rate = 2
        self.r = rospy.Rate(self.rate)
        self.linear_speed = 0.08
	self.angular_speed = 0.1
	self.orth_angle = 0

	self.right_wall_existed = False	
	self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback) 
	self.objects_center = 10000
	self.objects_left = 10000
	self.objects_right = 10000
	self.prev_objects_right = 10000
	self.nearness_ceiling = 0.7
	self.nearness_floor= 0.5
	self.translation_amount = 1
	self.rotation_amount = 5

	self.goal_pos = Point()
	self.goal_pos.x = 10
	self.goal_pos.y = 0
	self.distance_tolerance = 0.1
	self.angular_tolerance = radians(2.5)

	self.lock = False

	self.run_pathfinder()

    def get_odom(self):
	try:
	    (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
	except (tf.Exception, tf.ConnectivityException, tf.LookupException):
	    rospy.loginfo("TF Exception")
	    return 
	return (Point(*trans), quat_to_angle(Quaternion(*rot)))


    def run_pathfinder(self):
	print("Starting Pathfinder")
	self.mline = (self.goal_pos.y - self.start.y) / (self.goal_pos.x - self.start.x) 
	self.update_odom()
	print("Mline: " + str(self.mline)) 

	while not rospy.is_shutdown():
	    while abs(degrees(self.rot - atan2(self.goal_pos.y - self.start.y, self.goal_pos.x - self.start.x))) > 3 and not rospy.is_shutdown():
		#print("rotating to face goal")
		self.rotate_inc() 
	#	self.rotate_amt(-self.rot)
		self.update_odom()
	    	self.r.sleep()
	    while not self.obstacle_in_front(): 
	        self.translate_inc()
		self.r.sleep()
		self.update_odom()
		if self.goal_reached():
		    return True 
	    #self.print_dists()
	    print("Found obstacle at {}, {}".format(self.pos.x, self.pos.y))
	    self.cmd_vel.publish(Twist()) 
	    self.hitpt = self.pos
	    self.hitpt_rot = self.rot 
	    
	    if not self.follow_boundary():
		return False
	    if self.goal_reached():
		return True
	    #print("starting at mline again") 
	    self.r.sleep()
    
    def follow_boundary(self):
	print("Following boundary...") 
	self.message = ""
	while not rospy.is_shutdown():
	    #print("Rotating by: " + str(degrees(self.orth_angle + pi/2 - self.rot)) + " | " + self.message)
	    self.rotate_amt(self.orth_angle + pi/2 - self.rot)
            self.update_odom() 
	    if self.orth_angle == self.rot - pi/2 or self.orth_angle == self.rot - pi/2 - radians(15): #if forward or slightly to the right
		self.translate_inc()
		self.update_odom()
	    elif degrees(self.orth_angle + pi/2 - self.rot) > 0: #if any sort of left turn 
		self.translate_inc()
		self.translate_inc()
		self.update_odom()
	    #self.print_slope()
	    else:
		self.r.sleep()
		continue
 
	    if self.at_hitpt():
		print("Found hitpoint, returning false")
		return False
	    elif self.goal_reached() or self.is_mline():
		if self.euclidean_distance(self.pos, self.goal_pos) >= self.euclidean_distance(self.hitpt, self.goal_pos):
			return False 
		print("Found m-line")
		return True
	    self.r.sleep() 

	
    def at_hitpt(self):
	return self.euclidean_distance(self.pos, self.hitpt) < .001

    def obstacle_in_front(self):
	result = math.isnan(self.objects_center) == False and self.objects_center < 0.8
	return result

    def update_odom(self): 
	(self.pos, self.rot) = self.get_odom()

    def face_pos(self, target_pos):
	self.update_odom()
	print("Current angle: " + str(degrees(self.rot)))
	correct_angle = atan2(target_pos.y - self.pos.y, target_pos.x - self.pos.x)
	print("Correct angle to face: " + str(degrees(correct_angle)) + ", Rotating by: " + str(degrees(correct_angle - self.rot)))
	self.rotate_amt(correct_angle - self.rot)
	self.r.sleep()
	self.update_odom()
	print("Final rot: " + str(degrees(self.rot)))

    def print_slope(self):
	correct_angle = atan2(self.goal_pos.y - self.pos.y, self.goal_pos.x - self.pos.x)
	slope = (self.goal_pos.y - self.pos.y) / (self.goal_pos.x - self.pos.x)
	print("Slope to goal: " + str(slope) + ", Angle to goal: " + str(correct_angle))
	print("M line slope: " + str(self.mline)) 

    def rotate_amt(self, angle):
	self.lock = True
	move_cmd = Twist()
	angular_duration = math.fabs(angle / self.angular_speed)

	if angle < 0:
		move_cmd.angular.z = self.angular_speed * -1.0
	else:
		move_cmd.angular.z = self.angular_speed

	ticks = int(angular_duration * self.rate)
	for t in range(ticks):
		self.cmd_vel.publish(move_cmd)
		self.r.sleep()
	self.cmd_vel.publish(Twist())
	self.lock = False

    def old_rotate(self, angle):	
	last_angle = self.rot
	turn_angle = 0
	while abs(turn_angle + self.angular_tolerance) < abs(angle) and not rospy.is_shutdown():
	    self.rotate_inc()
	    self.update_odom() 
	    delta_angle = normalize_angle(self.rot - last_angle)
	    turn_angle += delta_angle
	    last_angle = self.rot
	self.cmd_vel.publish(Twist())

    def get_orth_angle(self):
	return pi/2 

    def goal_reached(self):
	return self.euclidean_distance(self.goal_pos, self.pos) < 0.01 

    def print_pos(self):
	(pos, rot) = self.get_odom()
	self.pos = pos
	self.rot = rot
	print("(x, y) = ({}, {})".format(pos.x, pos.y))
   
    def face_angle(self, angle):
	amt = angle - self.rot
	self.rotate_amt(amt) 	
	 
    def is_mline(self):
	slope = (self.goal_pos.y - self.pos.y) / (self.goal_pos.x - self.pos.x) 
	return abs(self.mline - slope) < 0.008 
    
    def rotate_inc(self): #rotate incrementally
	move_cmd = Twist()
	move_cmd.angular.z = self.angular_speed
	self.cmd_vel.publish(move_cmd)

    def translate_inc(self): #translate incrementally
	move_cmd = Twist()
	move_cmd.linear.x = self.linear_speed
	self.cmd_vel.publish(move_cmd)
	self.r.sleep()
	self.cmd_vel.publish(Twist())  

    def print_dists(self):
	print("distances: {} {} {}".format(self.objects_left, self.objects_center, self.objects_right))
 
    def scan_callback(self, msg): #it's only 30 degrees below and above the center. 
	self.objects_center = msg.ranges[len(msg.ranges)/2]
	self.objects_left = msg.ranges[len(msg.ranges)-1]
	self.objects_right = msg.ranges[0]
	if self.lock: 
	    return 

	if (math.isnan(self.objects_right) and math.isnan(self.objects_center)):
		self.move_forward = 0
		self.message = "turning right to go find object"
		self.orth_angle = self.rot - pi/2 - radians(30) #turn 30 deg right to go find the object 
	elif self.objects_center > 0 and self.objects_center < 0.8:
		if math.isnan(self.objects_right) or self.objects_right > 0.6: #something is in front, nothing on the right. 
			self.orth_angle = self.rot
			self.message = "turn hard left to avoid wall in front"
		elif self.objects_right <= 0.6:
			self.move_Foward = 2
			self.orth_angle = self.rot - radians(30) #turn 60 deg left to avoid running into wall 
			self.message = "turning left to avoid wall by 60 deg"
	elif self.objects_right < 0.6:
		self.move_forward = 0
		self.orth_angle = self.rot - radians(30) 
		self.message = "right wall too close, turning 60 left"
	elif self.objects_right >= 0.6 and self.objects_right <= 0.8:
		self.orth_angle = self.rot - pi/2
		self.message = "just move forward"
		self.move_forward = 2
	elif self.objects_right > 0.8:
		self.message = "Move forward but slightly to the right"
		self.orth_angle = self.rot - pi/2 - radians(15)
		self.move_forward = 1
	elif self.objects_center >= 0.8 and (math.isnan(self.objects_right) or self.objects_right > 0.8):
		self.message = "Lost the right wall" 
		self.orth_angle = self.rot - pi/2 - radians(30)
		self.move_foward = 0
	else: 
		self.message = "turn left"
		self.orth_angle = self.rot 

    def euclidean_distance(self, goal, pos):
	return sqrt(pow((goal.x - pos.x), 2) + 
		    pow((goal.y - pos.y), 2)) 
	
    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)
 
if __name__ == '__main__':
    try:
        Bug2()
    except Exception as e:
	print(e)
        rospy.loginfo("Bug2 node terminated.")

