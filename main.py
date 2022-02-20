#!/usr/bin/env python3

import sys
import rospy

# import your tools here
# tool to create a new training environment
from rl_env.env_reset import env_reset
# mock up to get the rod's position information
import moveit_commander
from input_process.detect_rod import rod_detection
# moveit motion planning tool
# from moveit.py import move_yumi


def main():
    rospy.init_node("wrap_wrap", anonymous=True)
    # initializing the world environment
    env_pub = env_reset()
    env_pub.publish(0.3, 0, 0.2, 0.02, 0.2)
    moveit_commander.roscpp_initialize(sys.argv)
    
    scene = moveit_commander.PlanningSceneInterface()
    detector = rod_detection(scene)
    rospy.sleep(3)
    
    
    print(detector.scene_add_rod(detector.rod_state))


if __name__ == '__main__':
    main()