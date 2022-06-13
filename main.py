#!/usr/bin/env python3

import sys
import rospy

# # import your tools here
# # tool to create a new training environment
# from utility.rl_env import env_reset
# # mock up to get the rod's position information
import moveit_commander
from utility.detect_rod      import rod_detection
# from utility.detect_cable    import cable_detection
from utility.workspace_ctrl  import move_yumi
from utility.jointspace_ctrl import joint_ctrl
from utility.path_generator  import path_generator
from utility.gripper_ctrl    import gripper_ctrl
from utility.add_marker      import marker
# # moveit motion planning tool
# # from moveit.py import move_yumi

from geometry_msgs.msg import Pose
from transforms3d import euler
import moveit_msgs.msg

from math import pi


def main():
    rospy.init_node("wrap_wrap", anonymous=True)

    pg = path_generator()
    gripper = gripper_ctrl()
    goal = marker()

    display_trajectory_publisher = rospy.Publisher(
        "/move_group/display_planned_path",
        moveit_msgs.msg.DisplayTrajectory,
        queue_size=20,
    )

    ##-------------------##
    ## initializing the moveit 
    moveit_commander.roscpp_initialize(sys.argv)
    scene = moveit_commander.PlanningSceneInterface()
    robot = moveit_commander.RobotCommander()
    ctrl_group = []
    ctrl_group.append(moveit_commander.MoveGroupCommander('left_arm'))
    ctrl_group.append(moveit_commander.MoveGroupCommander('right_arm'))

    # gripper_group = []
    # gripper_group.append(moveit_commander.MoveGroupCommander('left_gripper'))
    # gripper_group.append(moveit_commander.MoveGroupCommander('right_gripper'))

    ## initialzing the yumi motion planner
    yumi = move_yumi(robot, scene, ctrl_group)

    ##-------------------##
    ## reset the robot
    # gripper.l_open()
    # gripper.r_open()
    j_ctrl = joint_ctrl(ctrl_group)
    j_ctrl.robot_default_l()
    j_ctrl.robot_default_r_low()
    gripper.l_open()
    gripper.r_open()

    rospy.sleep(1)


    ## prepare to show obstacle in rviz
    rod = rod_detection(scene)
    rod.detect()
    rod.scene_add_rod(rod.rod_state)
    ## Need time to initializing
    rospy.sleep(3)

    # # rospy.sleep(10)

    ## TODO: left arm move out of the camera's fov
    pose_goal = Pose()
    ...
    
    # # here get the rod state (from gazebo)
    # print("rod's state is:", end = ':')
    # rod.rod_state
    # print('adding rod to rviz scene')
    # print(rod.scene_add_rod(rod.rod_state))

    # cable = cable_detection(100)
    # # get the links' state (from gazebo)
    # cable.get_links()

    # # find the link that is closest to the rod
    # # Use data on x-z plane
    # ptr = {'idx': 0, 'val':10e6}
    rod_x = rod.rod_state.position.x
    rod_y = rod.rod_state.position.y
    rod_z = rod.rod_state.position.z
    # for i in range(cable.no_of_links):
    #     link_pos = cable.links[i].link_state.pose.position

    #     cable_x = link_pos.x
    #     cable_z = link_pos.z
    #     val = (rod_x-cable_x)**2 + (rod_z-cable_z)**2
    #     if val < ptr['val']:
    #         ptr['idx'] = i
    #         ptr['val'] = val

    # print('contact section at: ', end = '')
    # print(ptr['idx'])

    # # ##-------------------##
    # # ## calculate which link to hold on to for the RIGHT hand
    # # ## the right hand will be used for keeping the cable in position
    # # ## so here an arbitary value will be given

    # # grabbing_point = ptr['idx'] - 4

    # # ## move the right end-effector to the target link
    # # #q = euler.euler2quat(pi, 0, pi/2, 'sxyz')

    # # cable.get_links()
    # # link1 = cable.links[grabbing_point].link_state.pose.position
    # # link2 = cable.links[grabbing_point-1].link_state.pose.position
    # # x = (link1.x + link2.x)/2
    # # y = (link1.y + link2.y)/2
    # # z = (link1.z + link2.z)/2
    # # start = [x, y - 0.12 - 0.25, z]
    # # yumi.go_to_pose_goal(ctrl_group[1], pose_goal)
    
    # # cable.get_links()
    # # link2pick = cable.links[grabbing_point].link_state.pose.position
    # # stop  = [x, y - 0.12, z]

    # # goal.show(x=stop[0], y=stop[1], z=stop[2])

    # # path = [start, stop]
    # # cartesian_plan, fraction = yumi.plan_cartesian_traj(ctrl_group, 1, path)
    # # yumi.execute_plan(cartesian_plan, ctrl_group[1])
    # # print("go to pose have the cable in between gripper: ", end="")

    # # ## left gripper grabs the link
    # # gripper.r_close()

    # # # #ctrl_group[0].set_end_effector_link('gripper_l_finger_l')
    # # # #eef_link = ctrl_group[0].get_end_effector_link()
    # # # #scene.attach_box(eef_link, box_name, touch_links=touch_links)
    

    ##-------------------##
    ## calculate which link to hold on to for the LEFT hand
    ## the left hand will be used for wrapping
    ## a section of the link is 0.04
    ## the r of the rod is given by the rod's state


    ## move the left end-effector to the target link
    # # pose_goal = geometry_msgs.msg.Pose()
    # #q = euler.euler2quat(pi, 0, -pi/2, 'sxyz')
    # link1 = cable.links[grabbing_point].link_state.pose.position
    # link2 = cable.links[grabbing_point+1].link_state.pose.position
    x = rod_x
    y = 0
    z = 0.15
    start = [x, y + 0.1+0.25, z]
    stop  = [x, y + 0.1, z]

    goal.show(x=stop[0], y=stop[1], z=stop[2])

    path = [start, stop]
    cartesian_plan, fraction = yumi.plan_cartesian_traj(ctrl_group, 0, path)
    yumi.execute_plan(cartesian_plan, ctrl_group[0])
    print("go to pose have the cable in between gripper: ", end="")

    # ## left gripper grabs the link
    # gripper.l_close()

    # ##-------------------##
    # ## generate spiral here
    # # s is the center of the rod
    spiral_params = [rod_x, rod_y, rod_z]
    # # g is the gripper's starting position
    gripper_states = stop
    path = pg.generate_spiral(spiral_params, gripper_states)
    pg.publish_waypoints(path)

    ## motion planning and executing
    cartesian_plan, fraction = yumi.plan_cartesian_traj(ctrl_group, 0, path)
    yumi.execute_plan(cartesian_plan, ctrl_group[0])

    # # left gripper releases the link
    # gripper.l_open()

    # # left arm move out of the camera's fov
    # yumi.go_to_pose_goal(yumi.ctrl_group[0], pose_goal)

if __name__ == '__main__':
    main()