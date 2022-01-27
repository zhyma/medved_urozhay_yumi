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
        # # test to plot a line.
        # for i in range(30):
        #     path.append([0.3, i/100, 0.4])

        
        # s for the center of the spiral
        # x (front-back), y (left-right), z (up-down).
        # x and z are used for spiral, y is used for advance
        s = [0.3, 0.1, 0.4]
        # g for the center of the gripper
        g = [0.6, 0.1, 0.42]
        # starting angle theta_0
        t_0 = 0.2
        # ending angle theta_end
        t_e = 0.2+pi*2
        n_samples = 20
        n = -1/2
        # offset theta_offset
        t_os = 0

        # r = a\theta^(-1/2)
        r_0 = np.sqrt((s[0]-g[0])**2+(s[2]-g[2])**2)
        a = r_0/np.power(t_0, n)
        for i in range(n_samples):
            dt = (t_e-t_0)*i/n_samples
            r = a*np.power((t_0+dt), n)
            t = -dt + t_os
            x = r*np.cos(t) + s[0]
            y = g[1] + 0.005*i
            z = r*np.sin(t) + s[2]
            path.append([x,y,z])
        
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

