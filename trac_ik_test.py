#!/usr/bin/env python3

from math import pi

from trac_ik_python.trac_ik import IK
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from transforms3d import euler

from threading import Lock

class yumi_ik():

    def __init__(self):
        # publish sequence
        self.pub_seq = [0,1,3,4,5,6,2]
        # right arm -> 0, then left arm -> 1
        self.joint = [[0.0]*7, [0.0]*7]
        self._joint_lock = Lock()

        self.ik_solver = []
        # or you can use "yumi_body", +0.1 to z axis
        self.ik_solver.append(IK("world", "gripper_r_base"))
        self.ik_solver.append(IK("world", "gripper_l_base"))

        self.joint_sub = rospy.Subscriber("/yumi/joint_states", JointState, self._joint_callback)
        
        pub_r = []
        pub_l = []
        for i in range(7):
            topic_name = '/yumi/joint_pos_controller_' + str(i+1) + '_r/command'
            pub_r.append(rospy.Publisher(topic_name, Float64, queue_size=10))
            topic_name = '/yumi/joint_pos_controller_' + str(i+1) + '_l/command'
            pub_l.append(rospy.Publisher(topic_name, Float64, queue_size=10))

        self.joint_pub = [pub_r, pub_l]

    def solve(self, side, start, t):
        # t stands for target
        ik_sol = self.ik_solver[side].get_ik(start, t[0], t[1], t[2], t[3], t[4], t[5], t[6])
        # print(ik_sol)
        return ik_sol

    def _joint_callback(self, msg):
        with self._joint_lock:
            for name, position, velocity in zip(msg.name, msg.position, msg.velocity):
                #_angles[name] = position
                str_split = name.split('_')
                if str_split[0] == 'yumi':
                    n = int(str_split[2])
                    side = str_split[3]
                    if side == 'r':
                        self.joint[0][n-1] = position
                    else:
                        self.joint[1][n-1] = position

    def move_arm(self, side, target):
        ik_sol = self.solve(side, self.joint[side], target)
        if ik_sol is None:
            print('No IK solution')
            return

        for i in range(7):
            val = ik_sol[self.pub_seq[i]]
            self.joint_pub[side][i].publish(val)


if __name__ == '__main__':
    rospy.init_node('yumi_test', anonymous=True)
    rospy.sleep(1)
    test_node = yumi_ik()
    # rospy.Rate(10)
    # while not rospy.is_shutdown():
    #     rospy.sleep()

    rospy.sleep(1)
    # 1 -> left arm
    # x, y, z, qx, qy, qz, qw
    q = euler.euler2quat(pi/2, 0, pi/2, 'sxyz')
    val = test_node.move_arm(1, [0.4, 0.3, 0.3, q[0], q[1], q[2], q[3]])
    ## call `rosrun tf tf_echo world gripper_l_base`
    # st_node.move_arm(1, val)

