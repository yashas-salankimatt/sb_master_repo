# import rospy
# from std_msgs.msg import Float32MultiArray
# import matplotlib.pyplot as plt
# import numpy as np

# # Define a callback function to receive the data from the rostopic
# def audio_callback(data):
#     # Convert the data to a numpy array
#     audio_data = np.array(data.data)
#     # Plot the amplitude of the audio data
#     plt.plot(audio_data)
#     plt.show()

# if __name__ == '__main__':
#     # Initialize the ROS node
#     rospy.init_node('audio_visualizer')
#     # Subscribe to the rostopic
#     rospy.Subscriber('/audio', Float32MultiArray, audio_callback)
#     # Spin the node to receive messages
#     rospy.spin()

import rospy
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

# Initialize the figure and axis objects
fig, ax = plt.subplots()
bars = ax.bar(range(2048), [0] * 2048)

# Define a callback function to receive the data from the rostopic
def audio_callback(data):
    # Convert the data to a numpy array
    audio_data = np.array(data.data)
    # Update the bar chart with the new audio data
    for i, bar in enumerate(bars):
        bar.set_height(audio_data[i])

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('audio_visualizer')
    # Subscribe to the rostopic
    rospy.Subscriber('/audio', Float32MultiArray, audio_callback)
    # Define the animation function to update the plot with new data
    def update(frame):
        return bars
    # Create the animation object and start the animation
    ani = FuncAnimation(fig, update, interval=10, blit=True)
    plt.show()
