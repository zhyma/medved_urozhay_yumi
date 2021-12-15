#!/usr/bin/env python3

import math
from math import pi
import numpy as np

from trac_ik_python.trac_ik import IK
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

# tf has been deprecated
#import tf
from transforms3d import euler

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from threading import Lock

class path_generator():

    def __init__(self):
        self.waypoint_pub = rospy.Publisher('yumi_waypoint', Path, queue_size=1, latch=True)

    def generate_path(self):
        path = []
        # test to plot a line.
        for i in range(30):
            path.append([0.3, i/100, 0.4])
        
        return path

    def publish_waypoints(self, path):
        """
        Publish the ROS message containing the waypoints
        """
        
        msg = Path()
        msg.header.frame_id = "world"
        msg.header.stamp = rospy.Time.now()

        for wp in path:
            pose = PoseStamped()
            pose.pose.position.x = wp[0]
            pose.pose.position.y = wp[1]
            pose.pose.position.z = wp[2]

            quat = euler.euler2quat(-pi/2, 0, pi/2, 'sxyz')
            pose.pose.orientation.x = quat[0]
            pose.pose.orientation.y = quat[1]
            pose.pose.orientation.z = quat[2]
            pose.pose.orientation.w = quat[3]
            msg.poses.append(pose)

        self.waypoint_pub.publish(msg)
        rospy.loginfo("Published {} waypoints.".format(len(msg.poses)))
        return 

if __name__ == '__main__':
    rospy.init_node('yumi_test', anonymous=True)
    rospy.sleep(1)
    pg_node = path_generator()
    path = pg_node.generate_path()
    pg_node.publish_waypoints(path)

    rospy.sleep(1)

