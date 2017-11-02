#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math

from twist_controller import Controller

DEBUG = False

class DBWNode(object):
	def __init__(self):
		rospy.init_node('dbw_node')

		vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
		fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
		brake_deadband = rospy.get_param('~brake_deadband', .1)
		decel_limit = rospy.get_param('~decel_limit', -5)
		accel_limit = rospy.get_param('~accel_limit', 1.)
		wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
		wheel_base = rospy.get_param('~wheel_base', 2.8498)
		steer_ratio = rospy.get_param('~steer_ratio', 14.8)
		max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
		max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

		# Placeholders 
		self.dbw_enabled = True # In real scenario, this may/will be false, to handle code logic accordingly
		self.current_velocity = None
		self.target_velocity = None
		self.prev_throttle = 0
		self.twist_cmd = None

		# Publishers
		self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
										 SteeringCmd, queue_size=1)
		self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
											ThrottleCmd, queue_size=1)
		self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
										 BrakeCmd, queue_size=1)

		# Subscribers
		rospy.Subscriber('/dbw_enabled', Bool, self.dbw_enabled_cb)
		rospy.Subscriber('/current_velocity', TwistStamped, 
						  self.current_velocity_cb)
		rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cmd_cb)

	   
		# Initialize `TwistController` object, pass relevant args and kwargs
		self.controller = Controller(wheel_base = wheel_base, steer_ratio = steer_ratio,
									 min_speed = 5., max_lat_accel = max_lat_accel,
									 max_steer_angle = max_steer_angle, 
									 decel_limit = decel_limit,
									 vehicle_mass = vehicle_mass, 
									 accel_limit = accel_limit,
									 brake_deadband = brake_deadband,
									 wheel_radius = wheel_radius,
									 fuel_capacity = fuel_capacity)

		# Loop Node until interrupt ..
		self.loop()

	def loop(self):
		# Rate at which loop is circled back .. 
		rate = rospy.Rate(40) # 50Hz
		
		# Until interrupt ..
		while not rospy.is_shutdown():
			## Since call-back calls are unpredictable, assert variables to avoid failure .. 
			if self.target_velocity is None or self.current_velocity is None:
				if DEBUG : 
					rospy.loginfo(" **** DBW Node, target_velocity or current_vel are Null ****") 
				continue

			## Call control from Controller instance, which computes
			## Throttle, Steering, and Brake parameters .. 
			throttle, brake, steering = self.controller.control(self.target_velocity.linear.x, 
																self.target_velocity.angular.z, 
																self.current_velocity.linear.x,
																self.current_velocity.angular.z,
																self.dbw_enabled)

			if DEBUG :
				rospy.loginfo("DBW -- > Throttle, Break, Steering : {}, {}, {}".format(throttle, brake, steering))

				rospy.loginfo("DBW Enabled ?? -- > {}".format(self.dbw_enabled))

			## Only if DBW is enabled publish, DBW may not be enabled or can be interrupted
			## in actual run (by assited driver to take comtrol of car on need basis)  
			if self.dbw_enabled: 
				self.publish(throttle, brake, steering)
			
			rate.sleep() ## Sleep until defined rate of hz above ... 

	## Helper method to publish throttle, brake and steer to respective nodes .. 
	def publish(self, throttle, brake, steer):
		tcmd = ThrottleCmd()
		tcmd.enable = True
		tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
		tcmd.pedal_cmd = throttle
		self.throttle_pub.publish(tcmd)

		scmd = SteeringCmd()
		scmd.enable = True
		scmd.steering_wheel_angle_cmd = steer
		self.steer_pub.publish(scmd)

		bcmd = BrakeCmd()
		bcmd.enable = True
		bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
		bcmd.pedal_cmd = brake
		self.brake_pub.publish(bcmd)

	## Call back funtion for /dbw_enabled
	def dbw_enabled_cb(self, BoolMsg):
		self.dbw_enabled = bool(BoolMsg.data)

	## Call back funtion for /current_velocity
	def current_velocity_cb(self, TwistStampedMsg):
		self.current_velocity = TwistStampedMsg.twist

	## Call back funtion for /twist_cmd
	def twist_cmd_cb(self, TwistStampedMsg):
		self.target_velocity = TwistStampedMsg.twist

# MAIN : Call DBWNode(), which will run indefinite until interrupt is
# trigerred ..
if __name__ == '__main__':
	DBWNode()
