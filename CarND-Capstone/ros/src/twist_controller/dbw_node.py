#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math

from twist_controller import Controller

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''
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
		self.dbw_enabled = True # Default DBW Enabled, need to handle this situation final as it will not be in the actual run..
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
									 min_speed = 0.0, max_lat_accel = max_lat_accel,
									 max_steer_angle = max_steer_angle, 
									 decel_limit = decel_limit,
									 vehicle_mass = vehicle_mass, 
									 accel_limit = accel_limit)

		# Loop Node until interrupt ..
		self.loop()

	def loop(self):
		# Rate at which loop is circled back .. 
		rate = rospy.Rate(50) # 50Hz
		
		# Until interrupt ..
		while not rospy.is_shutdown():
			# TODO: Get predicted throttle, brake, and steering using `twist_controller`
			# You should only publish the control commands if dbw is enabled
			# throttle, brake, steering = self.controller.control(<proposed linear velocity>,
			#                                                     <proposed angular velocity>,
			#                                                     <current linear velocity>,
			#                                                     <dbw status>,
			#                                                     <any other argument you need>)
			
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

			## Only if DBW is enabled process publish, DBW may not be enabled or can be interrupted
			## in actual run, by assited driver to take comtrol of car on need basis ..  
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
