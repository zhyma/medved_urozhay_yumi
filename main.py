#!/usr/bin/env python3

import rospy

# import your tools here
# tool to create a new training environment
from update_env import env_talker
# mock up to get the rod's position information
from detect_rod import rod_detection
# moveit motion planning tool
from moveit.py import move_yumi


def main():
    # initializing the world environment
    env_talker(0.3, 0, 0.3, 0.02, 0.1)
    rospy.sleep(3)
    detector = 


if __name__ == '__main__':
    main()