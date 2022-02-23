#! /usr/bin/env python

import rospy
from visualization_msgs.msg import Marker

def show_marker(pub, x=0, y=0, z=0):
  marker = Marker()

  marker.header.frame_id = "world"
  marker.header.stamp = rospy.Time.now()

  # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
  marker.type = Marker.SPHERE
  marker.id = 0

  # Set the scale of the marker
  marker.scale.x = 0.05
  marker.scale.y = 0.05
  marker.scale.z = 0.05

  # Set the color
  marker.color.r = 0.0
  marker.color.g = 1.0
  marker.color.b = 0.0
  marker.color.a = 0.5

  # Set the pose of the marker
  marker.pose.position.x = x
  marker.pose.position.y = y
  marker.pose.position.z = z
  marker.pose.orientation.x = 0.0
  marker.pose.orientation.y = 0.0
  marker.pose.orientation.z = 0.0
  marker.pose.orientation.w = 1.0

  pub.publish(marker)


if __name__ == '__main__':
  rospy.init_node('rviz_marker')
  # while not rospy.is_shutdown():
  marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size = 10)
  rospy.sleep(1.0)
  show_marker(marker_pub, x=1, y=1, z=1)
    # rospy.rostime.wallsleep(1.0)