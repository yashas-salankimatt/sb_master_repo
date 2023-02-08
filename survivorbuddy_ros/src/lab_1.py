#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import CompressedImage, Image
from moveit_msgs.msg import DisplayTrajectory

# Python 2/3 compatibility imports
import sys
import copy
import rospy
import geometry_msgs.msg
import moveit_commander


try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


def startle_callback(data):
    """
    Callback that implements the startle behavior.
    """
    if detect_noise(data):
        execute_behavior()


def detect_noise(data):
    """
    The perceptual schema.
    Args:
        data: audio data.
    Return:
        bool: alert
    """
    alert = False
    ##################
    # YOUR CODE HERE #
    ##################

    return alert


def execute_behavior():
    """
    The motor schema.
    Args:
        alert (bool): ALERT signal.
    """
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group_name = "survivor_buddy_head"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # joint value planning
    joint_goal = move_group.get_current_joint_values()
    ##################################
    # YOUR CODE HERE                 #
    # You may modify the lines below #
    ##################################
    joint_goal[0] =  # Enter a value
    joint_goal[1] =  # Enter a value
    joint_goal[2] =  # Enter a value
    joint_goal[3] =  # Enter a value

    move_group.go(joint_goal, wait=True)
    plan = move_group.plan()
    move_group.stop()

    display_trajectory = DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    pub.publish(display_trajectory)

    # execute plan
    move_group.execute(plan[1], wait=True)


if __name__ == "__main__":
    rospy.init_node("lab_1_node", anonymous=False)
    moveit_commander.roscpp_initialize(sys.argv)

    pub = rospy.Publisher(
        "/move_group/display_planned_path", DisplayTrajectory, queue_size=20
    )
    sub = rospy.Subscriber("/audio", Float32MultiArray, callback=startle_callback)
    rospy.loginfo("Node started.")

    rospy.spin()
