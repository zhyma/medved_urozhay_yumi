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
# moveit motion planning tool
# from moveit.py import move_yumi


def main():
    rospy.init_node("wrap_wrap", anonymous=True)
    # initializing the world environment
    env_pub = env_reset()
    env_pub.publish(0.3, 0, 0.2, 0.02, 0.2)
    moveit_commander.roscpp_initialize(sys.argv)
    
    scene = moveit_commander.PlanningSceneInterface()
    rod = rod_detection(scene)
    # Need time to initializing
    rospy.sleep(3)

    # left arm move out of the camera's fov
    
    # here get the rod state
    print(rod.scene_add_rod(rod.rod_state))

    cable = cable_detection()
    # get the links' state
    cable.get_links()

    # find the link that is closest to the rod
    # Use data on x-z plane
    ptr = {'idx': 0, 'val':10e6}
    rod_x = rod.rod_state.Position.x
    rod_z = rod.rod_state.Position.z
    for i in range(cable.len)
        cable_x = i.Pose.Position.x
        cable_z = i.Pose.Position.z
        val = (rod_x-cable_x)**2 + (rod_z-cable_z)**2
        if val < ptr['val']:
            ptr['idx'] = i
            ptr['val'] = val

    # calculate which link to hold on to

    # move the left end-effector to the target link

    # grab the link

    # generate spiral here

    # motion planning and executing

    # release the link

    # left arm move out of the camera's fov

if __name__ == '__main__':
    main()