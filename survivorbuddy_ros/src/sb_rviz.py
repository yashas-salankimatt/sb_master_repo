#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
import copy

movement_time_ms = 100

# global variable to store the joint positions
curr_sb_msg = [0, 0, 0, 0]
curr_sb_msg_time = -1
prev_sb_msg = [0, 0, 0, 0]
prev_sb_msg_time = -1
curr_sb_twist = TwistStamped()

# helper function to convert input from rads to degrees


def degrees(x):
    return x * 180 / 3.141592653589793


def radians(x):
    return x * 3.141592653589793 / 180


def sb_callback(msg):
    global curr_sb_msg, prev_sb_msg, curr_sb_twist, curr_sb_msg_time, prev_sb_msg_time
    if curr_sb_msg == msg:
        return
    prev_sb_msg = copy.deepcopy(curr_sb_msg)
    curr_sb_twist = msg
    prev_sb_msg_time = curr_sb_msg_time
    curr_sb_msg[0] = msg.twist.linear.x
    curr_sb_msg[1] = msg.twist.linear.y
    curr_sb_msg[2] = msg.twist.linear.z
    curr_sb_msg[3] = msg.twist.angular.x
    curr_sb_msg_time = rospy.get_time()
    print("Received joint positions: ", curr_sb_msg)


def return_interpolated_joint_state():
    global curr_sb_msg, prev_sb_msg, curr_sb_twist, curr_sb_msg_time, prev_sb_msg_time
    curr_time = rospy.get_time()
    if curr_sb_msg_time == -1 or prev_sb_msg_time == -1:
        return [radians(x) for x in curr_sb_msg]
    if curr_time - curr_sb_msg_time > movement_time_ms/1000:
        return [radians(x) for x in curr_sb_msg]
    time_diff = curr_time - curr_sb_msg_time
    joint_state = [0, 0, 0, 0]
    for i in range(4):
        joint_state[i] = prev_sb_msg[i] + \
            (curr_sb_msg[i] - prev_sb_msg[i]) * \
            time_diff / (movement_time_ms/1000)
    joint_state = [radians(x) for x in joint_state]
    return joint_state

# main function


def main():
    rospy.init_node('test_joint', anonymous=True)
    # set up the publisher to /joint_states
    global curr_sb_msg_time, prev_sb_msg_time
    curr_sb_msg_time = rospy.get_time()
    prev_sb_msg_time = rospy.get_time()
    pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
    # set up the subscriber to /sb_cmd_state that is a TwistStamped
    rospy.Subscriber('/sb_cmd_state', TwistStamped, sb_callback)
    rate = rospy.Rate(60)  # 10hz
    joint_state = JointState()
    joint_state.header.stamp = rospy.Time.now()
    joint_state.name = ['torso_joint',
                        'neck_swivel', 'head_tilt', 'head_nod']
    joint_state.position = [0, 0, 0, 0]
    pub.publish(joint_state)
    while not rospy.is_shutdown():
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = ['torso_joint',
                            'neck_swivel', 'head_tilt', 'head_nod']
        joint_state.position = return_interpolated_joint_state()
        pub.publish(joint_state)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
