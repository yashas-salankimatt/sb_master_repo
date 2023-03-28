#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
import copy

movement_time_ms = 100
num_robots = 4

# global variable to store the joint positions
curr_sb_msgs = [[0, 0, 0, 0] for _ in range(num_robots)]
curr_sb_msg_times = [-1] * num_robots
prev_sb_msgs = [[0, 0, 0, 0] for _ in range(num_robots)]
prev_sb_msg_times = [-1] * num_robots
curr_sb_twists = [TwistStamped() for _ in range(num_robots)]

# helper function to convert input from rads to degrees


def degrees(x):
    return x * 180 / 3.141592653589793


def radians(x):
    return x * 3.141592653589793 / 180


def sb_callback(msg, robot_id):
    global curr_sb_msgs, prev_sb_msgs, curr_sb_twists, curr_sb_msg_times, prev_sb_msg_times
    if curr_sb_msgs[robot_id] == msg:
        return
    prev_sb_msgs[robot_id] = copy.deepcopy(curr_sb_msgs[robot_id])
    curr_sb_twists[robot_id] = msg
    prev_sb_msg_times[robot_id] = curr_sb_msg_times[robot_id]
    curr_sb_msgs[robot_id][0] = msg.twist.linear.x
    curr_sb_msgs[robot_id][1] = msg.twist.linear.y
    curr_sb_msgs[robot_id][2] = msg.twist.linear.z
    curr_sb_msgs[robot_id][3] = msg.twist.angular.x
    curr_sb_msg_times[robot_id] = rospy.get_time()
    print(
        f"Received joint positions for robot {robot_id}: ", curr_sb_msgs[robot_id])


def return_interpolated_joint_state(robot_id):
    global curr_sb_msgs, prev_sb_msgs, curr_sb_twists, curr_sb_msg_times, prev_sb_msg_times
    curr_time = rospy.get_time()
    if curr_sb_msg_times[robot_id] == -1 or prev_sb_msg_times[robot_id] == -1:
        return [radians(x) for x in curr_sb_msgs[robot_id]]
    if curr_time - curr_sb_msg_times[robot_id] > movement_time_ms / 1000:
        return [radians(x) for x in curr_sb_msgs[robot_id]]
    time_diff = curr_time - curr_sb_msg_times[robot_id]
    joint_state = [0, 0, 0, 0]
    for i in range(4):
        joint_state[i] = prev_sb_msgs[robot_id][i] + \
            (curr_sb_msgs[robot_id][i] - prev_sb_msgs[robot_id]
             [i]) * time_diff / (movement_time_ms / 1000)
    joint_state = [radians(x) for x in joint_state]
    return joint_state

# main function


def main():
    rospy.init_node('test_joint', anonymous=True)
    global curr_sb_msg_times, prev_sb_msg_times
    curr_sb_msg_times = [rospy.get_time()] * num_robots
    prev_sb_msg_times = [rospy.get_time()] * num_robots

    pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

    # set up the subscribers for each robot
    subscribers = []
    for i in range(num_robots):
        topic_name = f'/sb_{i}_cmd_state'
        subscribers.append(rospy.Subscriber(
            topic_name, TwistStamped, sb_callback, callback_args=i))

    rate = rospy.Rate(60)  # 60hz

    while not rospy.is_shutdown():
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = []
        joint_state.position = []

        for i in range(num_robots):
            joint_state.name.extend([f'torso_joint_{i}',
                                    f'neck_swivel_{i}',
                                     f'head_tilt_{i}',
                                     f'head_nod_{i}'])
            joint_state.position.extend(return_interpolated_joint_state(i))

        pub.publish(joint_state)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
