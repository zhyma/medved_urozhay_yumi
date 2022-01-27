#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Pose

class GazeboLinkPose:
  link_name = ''
  link_pose = Pose()
  def __init__(self):
    self.states_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback)
    # self.states_sub = rospy.Subscriber("/gazebo/link_states", LinkStates, self.callback)
    self.pose_pub = rospy.Publisher("/gazebo/rod", Pose, queue_size = 10)

  def callback(self, data):
    try:
      idx = 0
      for i in range(len(data.name)):
        if data.name[i] == 'rod':
            idx = i

      self.link_pose = data.pose[idx]
    except ValueError:
      pass

if __name__ == '__main__':
  try:
    rospy.init_node('gazebo_link_pose', anonymous=True)
    gp = GazeboLinkPose()
    publish_rate = 10

    rate = rospy.Rate(publish_rate)
    while not rospy.is_shutdown():
      gp.pose_pub.publish(gp.link_pose)
      rate.sleep()

  except rospy.ROSInterruptException:
    pass
