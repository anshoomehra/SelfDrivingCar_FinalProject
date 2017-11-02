#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import math
from itertools import islice, cycle
import copy

# Number of waypoints we will publish.
LOOKAHEAD_WPS = 200 
DEBUG = False

# This index search waypoints ahead and behind present
# Also act as reference for Acceleration
SEARCH_INDEX = 40

'''
1 meter/s = 2.23694 mph
Since the value being passed to pursuit as meter/s, below equation would give 8.95 meter/s
which should result into 20.020258 mph as max speed
'''
MAX_ACCL = SEARCH_INDEX/2.23693
MAX_DECEL = 1.
MIN_VEL = 1.

# This buffer act as bias to stop at ditance ahead or behind stop line.
STOP_LINE_GAP = 3

class WaypointUpdater(object):
	def __init__(self):
		rospy.init_node('waypoint_updater')

	# Subscribers
		# Provides current pose statistics of the car..
		rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
		
		# Provides base waypoints provided by Udacity Simulator, published only once..  
		self.base_wp_sub = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

		# Provides waypoint of ahead traffic light with red signal ..
		rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

		# Provide linear and angular velocities
		rospy.Subscriber('/current_velocity', TwistStamped, 
						  self.current_velocity_cb)
		

	# Publishers
		# Publish computed final waypoints 
		self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
		
		# TODO: Add a subscriber for /obstacle_waypoint below

	# Other member variables you need below

		# TL Placholders
		self.tl_red_waypoint_idx = -1
		
		# Waypoints Placeholders
		self.base_waypoints = None
		self.number_of_base_waypoints = None

		# Pose Placeholders
		self.current_pose = None
		self.car_x = None
		self.car_y = None
		self.min_dist_from_car_idx = None

		# Velocity Placeholders
		self.current_velocity = None
		
		# Loop until interrupt is issued as closing simulator or Ctrl+C as examples
		#rospy.Rate(1)
		rospy.spin()

	'''
	Helper Method for Acceleration Logic
	From nearest waypoint to nearest waypoint + Lookahead index set defined max acceleration 
	'''
	def accelerate(self, lane):
		for idx in range(self.min_dist_from_car_idx, self.min_dist_from_car_idx+LOOKAHEAD_WPS):
			# Ensure index is not out range and is cyclic
			idx = idx % self.number_of_base_waypoints
			# Update the target velocity 
			self.set_waypoint_velocity(self.base_waypoints, idx , MAX_ACCL)
			# Add to the list of final waypoints 
			lane.waypoints.append(self.base_waypoints[idx])

		return lane

	'''
	Helper method to find Euclidean Distance
	Input: Position Vector for target and source
	Output: Distance in defined units (in this case meters) 
	'''
	def euclidean_distance3D(self, position1, position2):
		eucDst = lambda p1, p2 : math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2 + (p1.z - p2.z)**2)
		return eucDst(position1, position2)

	'''
	Helper Method for Deceleration Logic
	'''
	def decelerate(self, lane):
		last_wp = self.base_waypoints[self.tl_red_waypoint_idx+STOP_LINE_GAP]
		last_wp.twist.twist.linear.x = 0.

		# From nearest waypoint to traffic light stop point define linear deceleration 
		for idx in range ( self.tl_red_waypoint_idx+STOP_LINE_GAP, self.min_dist_from_car_idx, -1):
			wp = self.base_waypoints[idx]
			dist = self.euclidean_distance3D(wp.pose.pose.position, last_wp.pose.pose.position)
			vel = math.sqrt(2 * MAX_DECEL * dist)

			if vel < 1.:
				vel = 0.
			
			wp.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
			lane.waypoints.append(wp)			

		return lane

	# Helper method to get waypoint velocity
	def get_waypoint_velocity(self, waypoint):
		return waypoint.twist.twist.linear.x

	# Helper method to set waypoint velocity
	def set_waypoint_velocity(self, waypoints, waypoint, velocity):
		waypoints[waypoint].twist.twist.linear.x = velocity

	# Helper method to get Eucledian Distance with inputs as waypoints
	def distance(self, waypoints, wp1, wp2):
		dist = 0
		dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
		for i in range(wp1, wp2+1):
			dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
			wp1 = i
		return dist

	# Helper Method:
	# Compute Final Waypoints, and publish them to /final_waypoints node
	def send_final_waypoints(self):
		# Car's present position
		car_x = self.current_pose.position.x
		car_y = self.current_pose.position.y

		# Minimum Distance Placeholders
		min_dist_from_car = 99999
		
		# Local Placeholders 
		wp_start_idx = 0 # Index to compute waypoints from base list
		wp_end_idx = self.number_of_base_waypoints # Index until computing waypoints from base list 

		# Limit Search for nearest waypoint using search index for performance reasons
		if (self.min_dist_from_car_idx is not None):
			wp_start_idx = self.min_dist_from_car_idx - SEARCH_INDEX
			wp_end_idx = min( self.number_of_base_waypoints , self.min_dist_from_car_idx+SEARCH_INDEX )

		# Find minimum distance car and it's index
		for idx in range(wp_start_idx, wp_end_idx):
			waypoint = self.base_waypoints[idx]
			wp_x = waypoint.pose.pose.position.x
			wp_y = waypoint.pose.pose.position.y

			# Compute Distance : sqrt ( (x1-x2)^2 , (y1-y2)^2 )
			distance = math.sqrt( (car_x - wp_x)**2 + (car_y - wp_y)**2 )

			# Register minimum distance
			if distance < min_dist_from_car:
				min_dist_from_car = distance
				self.min_dist_from_car_idx = idx

		lane = Lane()
		
		# Logic deciding when to Accelerate vs Decelerate
		# Decelerate only if car is at distance of SEARCH_INDEX (meters)
		if self.tl_red_waypoint_idx is None or self.tl_red_waypoint_idx < 0:
			if DEBUG:
				rospy.loginfo("Accelerate")
			lane = self.accelerate(lane)
		else:
			if self.min_dist_from_car_idx >= self.tl_red_waypoint_idx:
				if DEBUG :
					rospy.loginfo("Accelerate")
				lane = self.accelerate(lane) 
			else:
				distance_to_tl = self.euclidean_distance3D(self.base_waypoints[self.min_dist_from_car_idx].pose.pose.position, self.base_waypoints[self.tl_red_waypoint_idx].pose.pose.position)
				if distance_to_tl > 0 and distance_to_tl < SEARCH_INDEX:
					if DEBUG:
						rospy.loginfo("Decelerate")
					lane = self.decelerate(lane)
				else:
					if DEBUG:
						rospy.loginfo("Accelerate")
					lane = self.accelerate(lane)

		# Waypoints, and velocities are set, time to Publish waypoints to /final_waypoints node ..
		if DEBUG :
			rospy.loginfo("Publishing next waypoints to final_waypoints")

		self.final_waypoints_pub.publish(lane)

	# Call Back Method for /current_pose:
	def pose_cb(self, PoseStampedMsg):
		
		if DEBUG :
			rospy.loginfo("In Pose CB...")

		self.current_pose = PoseStampedMsg.pose

		if DEBUG :
			rospy.loginfo("Current Pose {} , {}, {}".format(self.current_pose.position.x,
															self.current_pose.position.y, 
															self.current_pose.position.z))

		# Publish final waypoints ..
		# Wait until base_waypoits are published, call backs are not in sequence &
		# are unpredictable, need to assert required variables prior processing .. 
		if self.base_waypoints is not None: 
			self.send_final_waypoints()
 
	# Call Back Method for /base_waypoints: -- only publishes once in lifecycle
	def waypoints_cb(self, LaneMsg):

		if DEBUG :
			rospy.loginfo("In Waypoints CB...")

		self.base_waypoints = LaneMsg.waypoints
		self.number_of_base_waypoints = len(LaneMsg.waypoints)

		if DEBUG :
			rospy.loginfo("In Waypoints CB, total number of waypoints {}...".format(self.number_of_base_waypoints))

		# Unregister after receiving base-waypoints to gain performance
		self.base_wp_sub.unregister()
		self.base_wp_sub = None

	# Call Back Method for /traffic_waypoint
	def traffic_cb(self, msg):
		self.tl_red_waypoint_idx = msg.data 

	# Call Back Method for obstacle
	def obstacle_cb(self, msg):
		pass

	# Call back method for /current_velocity
	def current_velocity_cb(self, TwistStampedMsg):
		self.current_velocity = TwistStampedMsg.twist

# MAIN : Call WaypointUpdater(), which will run indefinite until interrupt is
# trigerred ..
if __name__ == '__main__':
	try:
		WaypointUpdater()
	except rospy.ROSInterruptException:
		rospy.logerr('Could not start waypoint updater node.')
