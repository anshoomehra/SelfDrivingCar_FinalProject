
from yaw_controller import YawController
from lowpass import LowPassFilter
from pid import PID
import rospy
import math

GAS_DENSITY = 2.858
ONE_MPH = 0.44704
MAX_SPEED = 30.0


class Controller(object):

	def __init__(self, *args, **kwargs):

		## Initialize Yaw Controller Object .. 
		self.yaw_controller = YawController(kwargs['wheel_base'], kwargs['steer_ratio'],
										 kwargs['min_speed'], kwargs['max_lat_accel'],
										 kwargs['max_steer_angle'])
		
		## Define PID, to be used later for Throttle computation
		self.throttle_pid = PID(kp=0.3, ki=0.0, kd=0.0, mn=kwargs['decel_limit'], mx=kwargs['accel_limit'])

		## Minimum Speed placeholder 
		self.min_speed = kwargs['min_speed']
		self.steer_ratio = kwargs['steer_ratio']

		## Placeholders for effective breaking .. 
		self.wheel_base = kwargs['wheel_base']
		self.wheel_radius = kwargs['wheel_radius']
		self.total_mass = kwargs['vehicle_mass'] + kwargs['fuel_capacity'] * GAS_DENSITY # And even air pressure would effect mass distrubution
		self.brake_deadband = kwargs['brake_deadband']
		
		## Time Logging Placeholder 
		self.prev_time = None

		### Define low pass filter & apply if needed
		self.s_lpf = LowPassFilter(tau = .2, ts = .1)
		
	# control method: for any preprocessing of throttle, brake, yaw
	def control(self, *args, **kwargs):

		# Retrieve present throttle, brake, steer
		target_velocity_linear_x = args[0]
		target_velocity_angular_z = args[1]
		current_velocity_linear_x = args[2]
		current_velocity_angular_z = args[3]
		dbw_enabled = args[4]
		
		throttle = 0.0
		brake = 0.0
		steering = 0.0

		## If DBW is interrupted, reset throttle to avoid errors & return .. 
		if not dbw_enabled:
			self.throttle.reset()
			self.steer_pid.reset()
			return 0, 0, 0

		# Compute difference between target and current velocity as CTE for throttle. 
		diff_velocity = target_velocity_linear_x - current_velocity_linear_x

		current_time = rospy.get_time()
		dt = 0
		
		# Compute Delta Time from last compute..
		if self.prev_time is not None: 
			dt = current_time - self.prev_time
		self.prev_time = current_time


		velocity_controller = 0
		
		# Determine if throttle or braking is needed .. 
		if dt > 0:
			velocity_controller = self.throttle_pid.step(diff_velocity, dt)

		if velocity_controller > 0: # Throttle
			throttle = velocity_controller
		elif velocity_controller < 0: # Braking
			brake = -velocity_controller
			
			# Check for dead-band defined in config
			if brake < self.brake_deadband:
				brake = 0.0
			
			# Apply other factors which impact braking intensity like vehcile mass, wheel radius, air pressure ..
			brake = brake * self.total_mass * self.wheel_radius * 4


		# Define yaw from yaw conrtoller, given target and present linear, angular velocities ..
		steering = self.yaw_controller.get_steering(target_velocity_linear_x, target_velocity_angular_z, current_velocity_linear_x)
		
		# Apply filter if needed
		#steering = self.s_lpf.filt(steering)
		
		return throttle, brake, steering

