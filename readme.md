![](bear-carrying-ear-of-corn-489x450.jpg)

Image source: [Bears Chow Down on $20,000 Worth of Maryland Corn](https://baltimorefishbowl.com/stories/bears-chow-20000-worth-maryland-corn/)

*A bear is harvesting corn. He gets one ear of corn, holds it with his left armpit. Then he reaches out to another ear of corn with his left paw, holds it with his right armpit. Then raising his right paw for another...At the end of the day, how many pieces of corn does he have?*

- Yumi's motion planning playground!
- Tested on ROS Noetic.
- Follow steps posted on [https://github.com/zhyma/yumi](https://github.com/zhyma/yumi) to set up the environment.

- Run `roslaunch yumi_gazebo yumi_gazebo_moveit.launch` to run Gazebo simulation (not necessary).
- Run `roslaunch yumi_moveit_config demo.launch` to run MoveIt.
- Run `moveit_test.py`. It will call `show_path_test.py` to generate a trajectory and then execute it.
	1. Go to the starting pose
	2. Run `show_path_test.py` to generate path
	3. Wait for 2 seconds
	4. Follow the generated trajectory