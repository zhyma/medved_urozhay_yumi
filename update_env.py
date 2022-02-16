#!/usr/bin/env python3

import rospy
from yumi_gazebo.msg import CylinderProperties


def env_talker(x, y, z, r, l):
    pub = rospy.Publisher('/set_rod_properties', CylinderProperties, queue_size = 10)
    rospy.init_node('update_env', anonymous = True)
    rospy.sleep(1.0)
    data = CylinderProperties()
    data.x = x
    data.y = y
    data.z = z
    data.r = r
    data.l = l
    pub.publish(data)

if __name__ == '__main__':
    env_talker(0.3, 0, 0.3, 0.02, 0.1)