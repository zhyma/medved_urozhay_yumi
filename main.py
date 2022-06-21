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
    gripper.l_open()
    gripper.r_open()
    j_ctrl = joint_ctrl(ctrl_group)
    j_ctrl.robot_default_l_low()
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

    ## TODO: left arm move out of the camera's fov
    pose_goal = Pose()

    rod_x = rod.rod_state.position.x
    rod_y = rod.rod_state.position.y
    rod_z = rod.rod_state.position.z

    x = rod_x
    y = 0
    z = 0.10
    start = [x+0.01, y + 0.1+0.25, z]
    stop  = [x+0.01, y + 0.1, z]

    goal.show(x=stop[0], y=stop[1], z=stop[2])

    path = [start, stop]
    cartesian_plan, fraction = yumi.plan_cartesian_traj(ctrl_group, 0, path)
    yumi.execute_plan(cartesian_plan, ctrl_group[0])
    print("go to pose have the cable in between gripper: ", end="")
    rospy.sleep(2)

    gripper.l_close()

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

    # path1 = path[0:len(path)//2]
    # path2 = path[len(path)//2:]

    ## motion planning and executing
    cartesian_plan, fraction = yumi.plan_cartesian_traj(ctrl_group, 0, path)
    ## fraction < 1: not successfully planned
    print(fraction)
    yumi.execute_plan(cartesian_plan, ctrl_group[0])
    rospy.sleep(2)

    gripper.l_open()
    gripper.r_open()

    start = path[-1]
    stop = [start[0], start[1], 0.1]

    path = [start, stop]
    cartesian_plan, fraction = yumi.plan_cartesian_traj(ctrl_group, 0, path)
    yumi.execute_plan(cartesian_plan, ctrl_group[0])


    

    

if __name__ == '__main__':
    main()