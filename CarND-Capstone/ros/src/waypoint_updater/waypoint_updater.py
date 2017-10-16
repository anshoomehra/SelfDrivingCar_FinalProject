#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math
from itertools import islice, cycle

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

    # Subscribers
        # Provides current pose statistics of the car..
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        # Provides base waypoints provided by Udacity Simulator, published only once..  
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

    # Publishers
        # Publish computed final waypoints 
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

    # Other member variables you need below
        
        # Waypoints Placeholders
        self.base_waypoints = None
        self.number_of_base_waypoints = None

        # Pose Placeholders
        self.current_pose = None
        self.car_x = None
        self.car_y = None

        # Minimum Distance Placeholders
        self.min_dist_from_car = 99999
        self.min_dist_from_car_idx = None

        # Velocity Placeholders
        self.current_velocity = None
        self.max_velocity = 10 # m/s
        
        # Loop until interrupt is issued as closing simulator or Ctrl+C as examples
        rospy.spin()


    # Compute velocity, and set each waypoint with target velocity .. 
    def update_waypoints_velocity(self, waypoints):
        for i in range(len(waypoints)):
            waypoints[i].twist.twist.linear.x = self.max_velocity

        return waypoints


    # Helper Method:
    # Compute Final Waypoints, and publish them to /final_waypoints node
    def send_final_waypoints(self):
        # Car's present position
        car_x = self.current_pose.position.x
        car_y = self.current_pose.position.y

        # Local Placeholders 
        wp_start_idx = 0 # Index to compute waypoints from base list
        wp_end_idx = self.number_of_base_waypoints # Index until computing waypoints from base list 

        # If this is true, it means it is not the first time this event has triggered
        # set the start and end index according to where car is, 20 before and either 
        # 20 after or number of total waypoints whichever is less .. 
        # instead of looping all waypoints to gain performance .. 
        if (self.min_dist_from_car_idx is not None):
            wp_start_idx = self.min_dist_from_car_idx - 20
            wp_end_idx = min( self.number_of_base_waypoints , self.min_dist_from_car_idx+20 )

        # Find minimum distance car and it's index
        for idx in range(wp_start_idx, wp_end_idx):
            waypoint = self.base_waypoints[idx]
            wp_x = waypoint.pose.pose.position.x
            wp_y = waypoint.pose.pose.position.y

            # Compute Distance : sqrt ( (x1-x2)^2 , (y1-y2)^2 )
            distance = math.sqrt( (car_x - wp_x)**2 + (car_y - wp_y)**2 )

            # Register minimum distance
            if distance < self.min_dist_from_car:
                self.min_dist_from_car = distance
                self.min_dist_from_car_idx = idx

        # Closest waypoint position ...
        closest_wp_pos = self.base_waypoints[self.min_dist_from_car_idx].pose.pose.position

        # Filter the waypoints which are ahead of the car
        # Caution: Since it is loop, waypoints are cyclic,
        #          hence using iterartor functions, cycle()
        #          & islice() to get elements from start of
        #          the index when list ends..
        wps_ahead = list(islice(cycle(self.base_waypoints), self.min_dist_from_car_idx, self.min_dist_from_car_idx + LOOKAHEAD_WPS))

        # Now since we have waypoints, update the target velocity for
        # each waypoint ..
        wps_with_velocity = self.update_waypoints_velocity(wps_ahead)

        # Waypoints, and velocities are set, time to Publish waypoints 
        # to /final_waypoints node ..
        rospy.loginfo("Publishing next waypoints to final_waypoints")
        
        lane = Lane()
        lane.waypoints = wps_with_velocity
        lane.header.stamp = rospy.Time(0)
        self.final_waypoints_pub.publish(lane)

    # Call Back Method for /current_pose:
    # Compute Final Waypoints, and publish them to /final_waypoints node
    def pose_cb(self, PoseStampedMsg):
        
        rospy.loginfo("In Pose CB...")
        # TODO: Implement

        self.current_pose = PoseStampedMsg.pose
        #Log Message Later ...
        rospy.loginfo("Current Pose {} , {}, {}".format(self.current_pose.position.x,
                                                        self.current_pose.position.y, 
                                                        self.current_pose.position.z))

        # Publish final waypoints ..
        ## Wait until base_waypoits are published, call backs are not in sequence &
        ## are unpredictable, need to assert required variables prior processing .. 
        if self.base_waypoints is not None: 
            self.send_final_waypoints()
 

    # Call Back Method for /base_waypoints:
    # Compute Final Waypoints, and publish them to /final_waypoints node
    def waypoints_cb(self, LaneMsg):

        rospy.loginfo("In Waypoints CB...")
        # TODO: Implement
        
        #Generate Waypoints (only ahead waypoints) based on present state
        #final_waypoints = self.generate_final_waypoints()

        # Log Message Later ...
        #print ("Current Velocity:", waypoints.twist.twist.linear.x, waypoints.twist.twist.linear.y, waypoints.twist.twist.linear.z)
        self.base_waypoints = LaneMsg.waypoints
        self.number_of_base_waypoints = len(LaneMsg.waypoints)

        rospy.loginfo("In Waypoints CB, total number of waypoints {}...".format(self.number_of_base_waypoints))

        # Publish final waypoints ..
        ## Wait until base_waypoits are published, call backs are not in sequence &
        ## are unpredictable, need to assert required variables prior processing .. 
        if self.current_pose is not None:
            self.send_final_waypoints()


    # ***** HAVE TO UPDATE CODE WITH BELOW HELPER METHODS ..
    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

# MAIN : Call WaypointUpdater(), which will run indefinite until interrupt is
# trigerred ..
if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
