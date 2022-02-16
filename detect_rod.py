#!/usr/bin/env python3

import math
from math import pi
import numpy as np

import rospy
from std_msgs.msg import Float64

# tf has been deprecated
#import tf
from transforms3d import euler

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from yumi_gazebo.msg import CylinderProperties


import sys
# import copy
# import rospy
import moveit_commander
# import moveit_msgs.msg
# import geometry_msgs.msg


class rod_detection():

    def __init__(self, scene):
        # You need to initializing a node before instantiate the class
        self.scene = scene
        self.rod_state = CylinderProperties()
        # directly get the true value from Gazebo
        rospy.Subscriber("/get_rod_properties",CylinderProperties, self.callback)

    def detect(self):
        ...

    def callback(self, data):
        self.rod_state.x = data.x
        self.rod_state.y = data.y
        self.rod_state.z = data.z
        self.rod_state.r = data.r
        self.rod_state.l = data.l


    def scene_add_rod(self, rod_info):
        cylinder_pose = PoseStamped()
        cylinder_pose.header.frame_id = "world"
        # assign cylinder's pose
        cylinder_pose.pose.position.x = rod_info.x
        cylinder_pose.pose.position.y = rod_info.y
        cylinder_pose.pose.position.z = rod_info.z
        cylinder_pose.pose.orientation.w = 1.0
        cylinder_name = "cylinder"
        # add_cylinder(self, name, pose, height, radius)
        self.scene.add_cylinder(cylinder_name, cylinder_pose, rod_info.l, rod_info.r)

        # ensuring collision updates are received
        start = rospy.get_time()
        seconds = rospy.get_time()
        timeout = 5
        while (seconds - start < timeout) and not rospy.is_shutdown():
            attached_objects = self.scene.get_attached_objects([cylinder_name])
            is_attached = len(attached_objects.keys()) > 0

            is_known = cylinder_name in self.scene.get_known_object_names()

            if (is_attached) and (is_known):
                return True

            rospy.sleep(0.1)
            seconds = rospy.get_time()
                
        return False


if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("rod_detector", anonymous=True)
    scene = moveit_commander.PlanningSceneInterface()
    detector = rod_detection(scene)

    rospy.sleep(5)
    print(detector.scene_add_rod(detector.rod_state))
