![](bear-carrying-ear-of-corn-489x450.jpg)

Image source: [Bears Chow Down on $20,000 Worth of Maryland Corn](https://baltimorefishbowl.com/stories/bears-chow-20000-worth-maryland-corn/)

*A bear is harvesting corn. He gets one ear of corn, holds it with his left armpit. Then he reaches out to another ear of corn with his left paw, holds it with his right armpit. Then raising his right paw for another...At the end of the day, how many pieces of corn does he have?*

- Yumi's motion planning playground!
- Tested on ROS Noetic.
- Follow steps posted on [https://github.com/zhyma/yumi](https://github.com/zhyma/yumi) to set up the environment.
- Need to install TRAC_IK (`sudo apt-get install ros-noetic-trac-ik`)
- Need to install transforms3d (`pip3 install transforms3d`)

## Motion planning test
- Run Gazebo
	Run `roslaunch yumi_gazebo mp_test.launch`
- To test motion planning script: 	
	Run `roslaunch yumi_moveit_config demo_gazebo.launch`

## With Simulated environment
- Run `roslaunch yumi_gazebo yumi_gazebo_moveit.launch` to run Gazebo simulation (not necessary).
- Run `roslaunch yumi_moveit_config demo_gazebo.launch` to run MoveIt.
- Run `main.py`.