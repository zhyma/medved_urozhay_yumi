#!/usr/bin/env python3

import sys
import rospy

# import your tools here
# tool to create a new training environment
from utility.rl_env import env_reset
# mock up to get the rod's position information
import moveit_commander
from utility.detect_rod import rod_detection
from utility.detect_cable import cable_detection
from utility.workspace_ctrl import move_yumi
from utility.jointspace_ctrl import robot_reset
from utility.path_generator import path_generator
from utility.gripper_ctrl import gripper_ctrl
# moveit motion planning tool
# from moveit.py import move_yumi

from geometry_msgs.msg import Pose
from transforms3d import euler

from math import pi


def main():
    rospy.init_node("wrap_wrap", anonymous=True)

    pg = path_generator()

    gripper = gripper_ctrl()

    ## initializing the moveit 
    moveit_commander.roscpp_initialize(sys.argv)
    scene = moveit_commander.PlanningSceneInterface()
    robot = moveit_commander.RobotCommander()
    ctrl_group = []
    ctrl_group.append(moveit_commander.MoveGroupCommander('left_arm'))
    ctrl_group.append(moveit_commander.MoveGroupCommander('right_arm'))

    ## initialzing the yumi motion planner
    yumi = move_yumi(robot, scene, ctrl_group)

    # currently joints space control is not working
    gripper.l_open()
    gripper.r_open()
    robot_reset(ctrl_group)
    gripper.l_open()

    ## initializing the world environment (gazebo plugin)
    env_pub = env_reset()
    env_pub.publish(x=0.3, y=0, z=0.4, r=0.02, l=0.05)

    ## prepare to show obstacle in rviz
    rod = rod_detection(scene)
    ## Need time to initializing
    rospy.sleep(3)

    
    # rospy.sleep(10)

    ## left arm move out of the camera's fov
    pose_goal = Pose()
    
    # here get the rod state (from gazebo)
    print("rod's state is:", end = ':')
    rod.rod_state
    print('adding rod to rviz scene')
    print(rod.scene_add_rod(rod.rod_state))

    cable = cable_detection(50)
    # get the links' state (from gazebo)
    cable.get_links()

    # find the link that is closest to the rod
    # Use data on x-z plane
    ptr = {'idx': 0, 'val':10e6}
    rod_x = rod.rod_state.position.x
    rod_z = rod.rod_state.position.z
    for i in range(cable.no_of_links):
        link_pos = cable.links[i].link_state.pose.position

        cable_x = link_pos.x
        cable_z = link_pos.z
        val = (rod_x-cable_x)**2 + (rod_z-cable_z)**2
        if val < ptr['val']:
            ptr['idx'] = i
            ptr['val'] = val

    print('contact section at: ', end = '')
    print(ptr['idx'])

    ## calculate which link to hold on to
    # a section of the link is 0.04
    # the r of the rod is given by the rod's state
    link_length = 0.04
    r = rod.rod_state.r
    need_links = int(3*pi*r/link_length)
    grabbing_point = ptr['idx'] + need_links
    print('grabing point is: ', end='')
    print(grabbing_point)

    ## move the left end-effector to the target link
    # pose_goal = geometry_msgs.msg.Pose()
    q = euler.euler2quat(pi, 0, -pi/2, 'sxyz')
    link2pick = cable.links[grabbing_point].link_state.pose.position
    start = [link2pick.x, link2pick.y + 0.12+0.25, link2pick.z]
    stop  = [link2pick.x, link2pick.y + 0.12, link2pick.z]

    path = [start, stop]
    cartesian_plan, fraction = yumi.plan_cartesian_traj(ctrl_group[0], path)
    yumi.execute_plan(cartesian_plan, ctrl_group[0])
    print("go to pose have the cable in between gripper: ", end="")

    ## left gripper grabs the link
    gripper.l_close()

    ## attach the object to the gripper

    ## generate spiral here
    # path = 

    ## motion planning and executing
    # cartesian_plan, fraction = yumi.plan_cartesian_traj(yumi.group_l, path)
    # yumi.execute_plan(cartesian_plan, yumi.group_l)

    ## left gripper releases the link

    ## left arm move out of the camera's fov
    # yumi.go_to_pose_goal(yumi.ctrl_group[0], pose_goal)

if __name__ == '__main__':
    main()