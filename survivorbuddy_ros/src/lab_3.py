#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import CompressedImage, Image

# Python 2/3 compatibility imports
import sys
import copy
import rospy
import geometry_msgs.msg

import cv2
##############################
# YOUR CODE HERE             #
##############################
# Import additional packages as per your requirement


try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String


class GestureRecognizer(object):
    """
    Gesture recognizer class.
    """
    def __init__(self):
        ##############################
        # YOUR CODE HERE             #
        # Create your publisher      #
        ##############################
        
        self.camera_sub = rospy.Subscriber("/camera/image/compressed", CompressedImage, callback=self.camera_callback, queue_size=1)
        rospy.loginfo("Node started.")

    def camera_callback(self, data):
        ################################
        # YOUR CODE HERE               #
        # Gesture recognition routines #
        ################################

if __name__ == '__main__':
    rospy.init_node("lab_3_node")
    ##############################
    # YOUR CODE HERE             #
    # Create your object(s)      #
    ##############################
    rospy.spin()
