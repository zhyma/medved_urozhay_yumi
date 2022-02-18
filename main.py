#!/usr/bin/env python3

import sys
import rospy

# import your tools here
# tool to create a new training environment
from env_update import env_update
# mock up to get the rod's position information
import moveit_commander
from detect_rod import rod_detection
# moveit motion planning tool
# from moveit.py import move_yumi


def main():
    rospy.init_node("wrap_wrap", anonymous=True)
    # initializing the world environment
    env_pub = env_update()
    env_pub.publish(0.3, 0, 0.2, 0.02, 0.2)
    rospy.sleep(3)
    
    moveit_commander.roscpp_initialize(sys.argv)
    
    scene = moveit_commander.PlanningSceneInterface()
    detector = rod_detection(scene)
    print(detector.scene_add_rod(detector.rod_state))


if __name__ == '__main__':
    main()