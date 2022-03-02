#! /usr/bin/env python

import rospy
from std_msgs.msg import Float64

class gripper_ctrl():
  def __init__(self):
    self.ctrl_l = rospy.Publisher("/yumi/gripper_effort_controller_l/command", Float64, queue_size = 10)
    self.ctrl_r = rospy.Publisher("/yumi/gripper_effort_controller_r/command", Float64, queue_size = 10)
    ...

  def l_close(self):
    self.ctrl_l.publish(-0.7)

  def r_close(self):
    self.ctrl_r.publish(-0.7)

  def l_open(self):
    self.ctrl_l.publish(10)

  def r_open(self):
    self.ctrl_r.publish(10)

if __name__ == '__main__':
  rospy.init_node('gripper_ctrl')
  g_ctrl = gripper_ctrl()
  rospy.sleep(1.0)
  g_ctrl.l_close()