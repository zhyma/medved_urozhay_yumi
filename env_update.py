#!/usr/bin/env python3

import rospy
from yumi_gazebo.msg import CylinderProperties

class env_update():
    def __init__(self):
        self.pub = rospy.Publisher('/set_rod_properties', CylinderProperties, queue_size = 10)
        rospy.sleep(1)

    def publish(self, x, y, z, r, l):
        data = CylinderProperties()
        data.position.x = x
        data.position.y = y
        data.position.z = z
        data.orientation.x = 0
        data.orientation.y = 0
        data.orientation.z = 0.707
        data.orientation.w = 0.707
        data.r = r
        data.l = l
        self.pub.publish(data)
        print("environment re-initialized!")

if __name__ == '__main__':
    rospy.init_node('update_env', anonymous = True)
    env_pub = env_update()
    env_pub.publish(0.3, 0, 0.3, 0.02, 0.1)