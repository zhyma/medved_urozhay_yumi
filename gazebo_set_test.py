#!/usr/bin/env python

import sys
import rospy
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Pose

class GazeboLinkPose:
  def __init__(self):
    self.mod = ''
    self.state = None
    self.read_flag = None
    self.states_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback)
    self.state_pub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size = 10)

  def callback(self, data):
    try:
      if self.mod != '':
        idx = -1
        for i in range(len(data.name)):
          if data.name[i] == self.mod:
            idx = i
        if idx == -1:
          print('no such a name')
        else:
          self.state = ModelState()
          self.state.model_name = data.name[idx]
          self.state.pose = data.pose[idx]
          self.state.twist = data.twist[idx]
          print(self.state)
          self.mod = ''
      
    except ValueError:
      pass

  def pose_update(self, data):
    self.mod = data[0]
    while self.state is None:
        pass

    if len(data)>=4:
        self.state.pose.position.x = data[1]
        self.state.pose.position.y = data[2]
        self.state.pose.position.z = data[3]

    if len(data)>=7:
        #should change orientation here
        pass

    self.state_pub.publish(self.state)
    self.state = None

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
