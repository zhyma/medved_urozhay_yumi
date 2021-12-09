#!/usr/bin/env python3

from trac_ik_python.trac_ik import IK
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from threading import Lock

_joint_lock = Lock()

# publish sequence
pub_seq = [0,1,3,4,5,6,2]
joint_l = [0.0]*7

target = [-2.0750, -1.0901, -0.0198, -0.3641, -0.4543, 1.4391, 1.6461]

def _joint_callback(msg):
    with _joint_lock:
        for name, position, velocity in zip(msg.name, msg.position, msg.velocity):
            #_angles[name] = position
            str_split = name.split('_')
            if str_split[0] == 'yumi':
                n = int(str_split[2])
                side = str_split[3]
                if side == 'l':
                    joint_l[n-1] = position
                else:
                    # right arm
                    ...

def listener():
    ik_solver = IK("yumi_body", "gripper_l_base")

    rospy.Subscriber("/yumi/joint_states", JointState, _joint_callback)

    if joint_l[0] == 0:
        rospy.sleep(0.1)
    
    # # No solution
    # ik_sol = ik_solver.get_ik(joint_l,
    #                        0.5103, -0.0327, 0.3197, # x, y, z
    #                        -0.0675, 0.8749, -0.3759, 0.2977) # QX, QY, QZ, QW

    ik_sol = ik_solver.get_ik(joint_l,
                           0.5903, 0.1418, 0.3692, # x, y, z
                           -0.0679, 0.8752, -0.3754, 0.2975) # QX, QY, QZ, QW


    return ik_sol
    # rospy.spin()

def talker():
    pub = []
    for i in range(7):
        topic_name = '/yumi/joint_pos_controller_' + str(i+1) + '_l/command'
        pub.append(rospy.Publisher(topic_name, Float64, queue_size=10))

    rospy.init_node('yumi_test', anonymous=True)
    rospy.sleep(1)

    ik_sol = listener();
    if ik_sol is None:
        print('No IK solution')
        return
    print(len(ik_sol), end =': ')
    print(ik_sol)
    

    for i in range(7):
        print(i+1, end=': ')
        val = ik_sol[pub_seq[i]]
        print(val)
        pub[i].publish(val)


if __name__ == '__main__':
    talker()