#!/usr/bin/env python3

from trac_ik_python.trac_ik import IK
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from threading import Lock

_joint_lock = Lock()


joints_val = [-1.4069, -2.0969, 0.2969, 0, 0, 0, 0.7069]


def robot_reset():
    # pub = rospy.Publisher('/yumi/joint_pos_controller_' + str(0+1) + '_l/command', Float64, queue_size=10)
    pub = []
    for i in range(7):
        topic_name = '/yumi/joint_pos_controller_' + str(i+1) + '_l/command'
        pub.append(rospy.Publisher(topic_name, Float64, queue_size=10))
        topic_name = '/yumi/joint_pos_controller_' + str(i+1) + '_r/command'
        pub.append(rospy.Publisher(topic_name, Float64, queue_size=10))

    for i in range(7*2):
        if i == 1 or i == 13:
            # left
            pub[i].publish(-joints_val[i//2])
        else:
            pub[i].publish(joints_val[i//2])

    rospy.sleep(1)


if __name__ == '__main__':
    rospy.init_node('yumi_test', anonymous=True)
    rospy.sleep(1)
    robot_reset()