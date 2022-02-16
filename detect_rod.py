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


# import sys
# import copy
# import rospy
import moveit_commander
# import moveit_msgs.msg
# import geometry_msgs.msg


class rod_detction():

    def __init__(self, scene):
        self.scene = scene

        ...

    def detect(self):
        ...

    def rod_embodied(self, pose, h, r):
        cylinder_pose = geometry_msgs.msg.PoseStamped()
        cylinder_pose.header.frame_id = "world"
        # assign cylinder's pose
        cylinder_pose.pose.orientation.w = 1.0
        cylinder_name = "cylinder"
        # add_cylinder(self, name, pose, height, radius)
        self.scene.add_cylinder(cylinder_name, cylinder_pose, h, r)
                
        return path
