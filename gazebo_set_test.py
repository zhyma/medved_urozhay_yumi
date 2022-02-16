#!/usr/bin/env python

import sys
import rospy
from gazebo_msgs.srv import GetModelState, SetModelState
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Pose

class GazeboLinkPose:
    def __init__(self):
        self.mod = ''
        self.state = None
        self.read_flag = None

    def pose_update(self, data):
        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

            resp = get_state(model_name=data[0])
            print(resp.pose)
            if len(data)>=4:
                resp.pose.position.x = data[1]
                resp.pose.position.y = data[2]
                resp.pose.position.z = data[3]

            # # if len(data)>=7:
            # #     #should change orientation here
            # #     pass

            rospy.wait_for_service('/gazebo/set_model_state')
            try:
                set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
                state = ModelState()
                state.model_name = data[0]
                state.pose = resp.pose
                print(state)
                resp = set_state(state)

            except rospy.ServiceException as e:
                pass
        except rospy.ServiceException as e:
            pass



if __name__ == '__main__':
    try:
        rospy.init_node('gazebo_link_pose', anonymous=True)
        gp = GazeboLinkPose()
        publish_rate = 10

        rate = rospy.Rate(publish_rate)
        run_flag = True
        while (not rospy.is_shutdown()) and run_flag:
            user_input = input('model_name, x, y, z, (optional: r, p, y):')
            input_list = user_input.replace(' ', '').split(',')
            print(input_list)
            if 'exit' in input_list[0]:
                run_flag = False
            else:
                pose = [input_list[0]]
                for i in range(1, len(input_list)):
                    pose.append(float(input_list[i]))

            print(pose)
            gp.pose_update(pose)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
