#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.camera_image = None
        self.lane = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()

    def pose_cb(self, PoseStampedMsg):
        self.pose = PoseStampedMsg

    def waypoints_cb(self, LaneMsg):
        self.lane = LaneMsg

    def traffic_cb(self, TrafficLightArrayMsg):
        self.lights = TrafficLightArrayMsg.lights

    def image_cb(self, ImageMsg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = ImageMsg
        light_wp, state = self.process_traffic_lights()

        #rospy.loginfo("Light WP & State: {}, {}".format(light_wp, state))
        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if (state in [TrafficLight.RED, TrafficLight.YELLOW] ) else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
        return 0

    def get_closest_index(self, car_pose, tl_pose_list):
        """Identifies the closest lights to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            car_pose: Position to match a waypoint from..
            tl_pose_list: List of traffic light positions..
        Returns:
            int: index of the closest waypoint..
        """
        #TODO implement
        min_dist = 1e100
        index = 0
        
        for i, lt in enumerate(tl_pose_list):
            dist = math.hypot(lt.pose.pose.position.x-car_pose.position.x, lt.pose.pose.position.y-car_pose.position.y)
            if dist < min_dist:
                min_dist = dist
                index = i
        return index

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification & return the same ..
        return self.light_classifier.get_classification(cv_image)

    def generate_light(self, x, y, z):
        light = TrafficLight()
        light.pose = PoseStamped()
        light.pose.pose.position.x = x
        light.pose.pose.position.y = y
        light.pose.pose.position.z = z
        
        return light

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        # Later move this back to init
        stop_line_positions = self.config['stop_line_positions']
        
        #if(self.pose):
        #    car_position = self.get_closest_waypoint(self.pose.pose)

        #TODO find the closest visible traffic light (if one exists)
        if(self.pose and self.lane):
            # Get closest Light Position
            light_position = self.get_closest_index(self.pose.pose, self.lights)
           
            # Using closest light position, get closest waypoint for the LIGHT
            light_wp = self.get_closest_index(self.lights[light_position].pose.pose, self.lane.waypoints)
            
            # Add all lights stop-line pose from config to lines list..
            # Perhaps we should only do it once .. 
            lines = list()
            for light_pos in stop_line_positions:
                light =  self.generate_light(light_pos[0], light_pos[1], 0.)
                lines.append(light)
            
            # Above we derived closest light waypoint, here we are deriving closest
            # Stop-Line waypoint ..
            line_wp = self.get_closest_index(lines[light_position].pose.pose, self.lane.waypoints)
            
            ## Let's get State for closest light  .. 
            ## rospy.loginfo("State information from traffic_lights: {}".format(self.lights[light_position].state))
            state = self.lights[light_position].state #self.get_light_state(self.lights[light_position])
            #rospy.loginfo_throttle(2, "Light: " + str(state))
            return line_wp, state

        if light:
            state = self.get_light_state(light)
            return light_wp, state
        
        self.waypoints = None
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
