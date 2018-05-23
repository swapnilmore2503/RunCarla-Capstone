#!/usr/bin/env python
# Team: RunCarla
# 

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

from scipy.spatial import KDTree

import math

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



'''
Instructions from lesson:

1. First Version 
The goal for the first version of the node should be simply to subscribe to the topics

/base_waypoints & /current_pose

and publish a list of waypoints to /final_waypoints
The /base_waypoints topic publishes a list of all waypoints for the track, 
so this list includes waypoints both before and after the vehicle 
(note that the publisher for /base_waypoints publishes only once). 
For this step in the project, the list published to /final_waypoints should include just a 
fixed number of waypoints currently ahead of the vehicle.

Topic : Msg Type
/base_waypoints : styx_msgs/Lane  
/current_pose  : geometry_msgs/PoseStamped 
/final_waypoints  :  styx_msgs/Lane


2. Final version


'''

# Rate in Hz
PUBLISHING_RATE = 50
LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb, queue_size=1)
        rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb, queue_size=1 )

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below

        self.pose = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None
        # Gives us control over the publishing frequency
        self.loop()

    # Infinite loop
    # First Version: Grabs the closest waypoint and publishes list
    def loop(self):
        rate = rospy.Rate(PUBLISHING_RATE)
        while not rospy.is_shutdown():
            if self.pose and self.base_waypoints:
                # Get the closest way-point
                closest_waypoint_idx = self.get_closest_waypoint_idx()
                self.publish_waypoints(closest_waypoint_idx)
            rate.sleep()


    def get_closest_waypoint_idx(self):
        # Current pose (x,y)
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        # Get the index of the closest way point
        closest_idx = self.waypoint_tree.query([x,y], 1)[1]

        # Check if point is infront or behind the car
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx - 1]

        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x,y])

        val = np.dot(cl_vect - prev_vect, pos_vect - cl_vect)

        if val > 0:
            closest_idx = (closest_idx+1) % len(self.waypoints_2d)

        return closest_idx

            
    def publish_waypoints(self, closest_idx):
        lane = Lane()
        lane.header = self.base_waypoints.header
        lane.waypoints = self.base_waypoints.waypoints[closest_idx : closest_idx+LOOKAHEAD_WPS]
        self.final_waypoints_pub.publish(lane)


    def pose_cb(self, msg):
        self.pose = msg


    # Latched callback, only called once (base waypoints are static)
    def waypoints_cb(self, waypoints):
        self.base_waypoints = waypoints
        # Setup the Kd Tree which has log(n) complexity
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)


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


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
