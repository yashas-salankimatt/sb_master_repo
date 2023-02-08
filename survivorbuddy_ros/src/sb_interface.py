import rospy
from geometry_msgs.msg import TwistStamped
from moveit_msgs.msg import ExecuteTrajectoryActionGoal

goalPos = []
twist = TwistStamped()

# helped function to convert input from rads to degrees
def degrees(x):
    return x * 180 / 3.141592653589793

# goal callback function
def goalCallback(msg):
    goalPos = msg.goal.trajectory.joint_trajectory.points[-1].positions
    # take all the values in goalPos and turn them from rads to degrees
    goalPos = [degrees(x) for x in goalPos]
    print(goalPos)
    twist.twist.linear.x = goalPos[0]
    twist.twist.linear.y = goalPos[1]
    twist.twist.linear.z = goalPos[2]
    twist.twist.angular.x = goalPos[3]

# main function
def main():
    rospy.Subscriber('/execute_trajectory/goal', ExecuteTrajectoryActionGoal, goalCallback)
    pub = rospy.Publisher('/sb_cmd_state', TwistStamped, queue_size=10)
    rospy.init_node('send_sb', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pub.publish(twist)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass