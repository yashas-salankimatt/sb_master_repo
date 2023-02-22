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


# parameters
RATE = 10 # ROS looping rate


class SchemaSB:
    def __init__(self, rate=RATE) -> None:
        """
        Inititialize Schema for SurvivorBuddy object
        Args:
            rate: int
                ROS rate, default = RATE
        Return:
            None
        """
        self.rate = rospy.Rate(RATE)

        self.audio_data = None
        self.pub = rospy.Publisher(
            "/move_group/display_planned_path", DisplayTrajectory, queue_size=20
        )
        self.sub = rospy.Subscriber(
            "/audio", Float32MultiArray, callback=self.audio_callback,
            queue_size=1
        )

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        group_name = "survivor_buddy_head"
        self.move_group = moveit_commander.MoveGroupCommander(group_name)


    def audio_callback(self, data):
        """
        Callback for audio.
        Args:
            data : Float32MultiArray
                audio data from mic
        Return:
            None
        """
        self.audio_data = data.data


    def detect_noise(self):
        """
        The perceptual schema.
        Args:
            None
        Return:
            alert : bool
                whether an alert was detected
        """
        alert = False
        ##################
        # YOUR CODE HERE #
        ##################

        return alert


    def execute_behavior(self):
        """
        The motor schema.
        Args:
            None
        Return:
            None
        """

        # joint value planning
        joint_goal = self.move_group.get_current_joint_values()
        ##################################
        # YOUR CODE HERE                 #
        # You may modify the lines below #
        ##################################

        joint_goal[0] =  # Enter a value
        joint_goal[1] =  # Enter a value
        joint_goal[2] =  # Enter a value
        joint_goal[3] =  # Enter a value

        # Either use move_group.go or use plan and execute, not both
        # self.move_group.go(joint_goal, wait=True)

        plan = self.move_group.plan(joint_goal)

        display_trajectory = DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan[1])
        self.pub.publish(display_trajectory)

        # execute plan
        self.move_group.execute(plan[1], wait=True)
        self.move_group.stop()


    def startle_loop(self):
        """
        Implement the startle behavior.
        Args:
            None
        Return:
            None
        """
        while not rospy.is_shutdown(): # loop until ROS node is shutdown
            if self.detect_noise():
                self.execute_behavior()
            self.rate.sleep() # delay sufficiently to maintain the ROS rate


if __name__ == "__main__":
    rospy.init_node("lab_1_node", anonymous=False)
    moveit_commander.roscpp_initialize(sys.argv)
    
    rospy.loginfo("Node started.")

    schema_sb = SchemaSB()
    schema_sb.startle_loop()