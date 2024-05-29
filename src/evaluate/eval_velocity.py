import rosbag
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock
import numpy as np

# Function to extract data from rosbag
def extract_data(bag_file, velocity_topic, clock_topic):
    time_data = []
    velocity_x_data = []
    velocity_y_data = []
    velocity_z_data = []
    velocity_magnitude_data = []

    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=[velocity_topic]):
            if topic == velocity_topic:
                # Extract time directly from message timestamp
                current_time = t.to_sec()

                # Extract velocity components from the specified topic
                velocity_x = msg.velocity.linear.x
                velocity_y = msg.velocity.linear.y
                velocity_z = msg.velocity.linear.z
                velocity_magnitude = np.sqrt(velocity_x**2 + velocity_y**2 + velocity_z**2)

                time_data.append(current_time)
                velocity_x_data.append(velocity_x)
                velocity_y_data.append(velocity_y)
                velocity_z_data.append(velocity_z)
                velocity_magnitude_data.append(velocity_magnitude)
    
    return time_data, velocity_x_data, velocity_y_data, velocity_z_data, velocity_magnitude_data

# Function to plot velocity vs time in a single square window
def plot_velocity(time_data, velocity_x_data, velocity_y_data, velocity_z_data, velocity_magnitude_data):
    fig, axs = plt.subplots(2, 1, figsize=(10, 10))

    # Plot velocity components (x, y, z) on the same graph
    axs[0].plot(time_data, velocity_x_data, label='Velocity X', color='r')
    axs[0].plot(time_data, velocity_y_data, label='Velocity Y', color='g')
    axs[0].plot(time_data, velocity_z_data, label='Velocity Z', color='b')
    axs[0].set_xlabel('Time (s)')
    axs[0].set_ylabel('Velocity (m/s)')
    axs[0].set_title('Velocity Components vs Time')
    axs[0].legend(loc='lower right')

    # Plot velocity magnitude
    axs[1].plot(time_data, velocity_magnitude_data, label='Velocity Magnitude', color='b')
    axs[1].set_xlabel('Time (s)')
    axs[1].set_ylabel('Velocity (m/s)')
    axs[1].set_title('Velocity Magnitude vs Time')
    axs[1].legend(loc='lower right')

    plt.tight_layout()
    plt.show()

# Main function
def main():
    bag_file = '/home/joaomendes/workspace1/src/motion_planning_python/data/tmp_bags/tmp_bag_2024-05-23-10-53-33.bag'
    velocity_topic = '/uav1/estimation_manager/uav_state'
    clock_topic = '/clock'

    # Extract data
    time_data, velocity_x_data, velocity_y_data, velocity_z_data, velocity_magnitude_data = extract_data(bag_file, velocity_topic, clock_topic)

    # Plot data
    plot_velocity(time_data, velocity_x_data, velocity_y_data, velocity_z_data, velocity_magnitude_data)

if __name__ == "__main__":
    main()
