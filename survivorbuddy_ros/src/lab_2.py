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

import cv2

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String


class GenericBehavior(object):
    """
    Generic behavior class.
    """
    def __init__(self):
        self.pub = rospy.Publisher(
            "/move_group/display_planned_path", DisplayTrajectory, queue_size=20
        )
        self.audio_sub = rospy.Subscriber("/audio", Float32MultiArray, callback=self.callback_1, queue_size=1)
        self.camera_sub = rospy.Subscriber("/camera/image/compressed", Float32MultiArray, callback=self.callback_2, queue_size=1)
        rospy.loginfo("Node started.")

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        self.group_name = "survivor_buddy_head"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)

    def callback_1(self, data):
        pass

    def callback_2(self, data):
        pass

if __name__ == '__main__':
    rospy.init_node("lab_1_node")
    moveit_commander.roscpp_initialize(sys.argv)
    ##############################
    # YOUR CODE HERE             #
    # Call the behavior(s)       #
    ##############################
    rospy.spin()
