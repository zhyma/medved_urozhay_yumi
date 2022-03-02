#!/usr/bin/env python3

import sys

from trac_ik_python.trac_ik import IK
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

import moveit_commander
import moveit_msgs.msg
from moveit_commander.conversions import pose_to_list




def robot_reset(ctrl_group):
    r_joints_val = [ 1.4069, -2.0969, -0.7069, 0.2969, 0, 0, 0]
    l_joints_val = [-1.4069, -2.0969,  0.7069, 0.2969, 0, 0, 0]

    ctrl_group[0].set_joint_value_target(l_joints_val)
    ctrl_group[1].set_joint_value_target(r_joints_val)
    ctrl_group[0].go(wait=True)
    ctrl_group[1].go(wait=True)
    # # pub = rospy.Publisher('/yumi/joint_pos_controller_' + str(0+1) + '_l/command', Float64, queue_size=10)
    # pub = []
    # for i in range(7):
    #     topic_name = '/yumi/joint_pos_controller_' + str(i+1) + '_l/command'
    #     pub.append(rospy.Publisher(topic_name, Float64, queue_size=10))
    #     topic_name = '/yumi/joint_pos_controller_' + str(i+1) + '_r/command'
    #     pub.append(rospy.Publisher(topic_name, Float64, queue_size=10))

    # for i in range(7*2):
    #     if i == 1 or i == 13:
    #         # left
    #         pub[i].publish(-joints_val[i//2])
    #     else:
    #         pub[i].publish(joints_val[i//2])

    # rospy.sleep(1)


if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('yumi_test', anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    ctrl_group = []
    ctrl_group.append(moveit_commander.MoveGroupCommander('left_arm'))
    ctrl_group.append(moveit_commander.MoveGroupCommander('right_arm'))


    robot_reset(ctrl_group)