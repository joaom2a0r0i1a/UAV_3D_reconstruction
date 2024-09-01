import rosbag
import matplotlib.pyplot as plt
from mrs_msgs.msg import UavState
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock
import numpy as np

# Function to extract data from rosbag
def extract_data(bag_file, state_topic, time_limit=2000):
    time_data = []
    velocity_x_data = []
    velocity_y_data = []
    velocity_z_data = []
    velocity_magnitude_data = []

    acceleration_x_data = []
    acceleration_y_data = []
    acceleration_z_data = []
    acceleration_magnitude_data = []

    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=[state_topic]):
            if topic == state_topic:
                current_time = t.to_sec()

                # Limit time in case simulation finishes early
                if current_time > time_limit:
                    break

                velocity_x = msg.velocity.linear.x
                velocity_y = msg.velocity.linear.y
                velocity_z = msg.velocity.linear.z
                velocity_magnitude = np.sqrt(velocity_x**2 + velocity_y**2 + velocity_z**2)

                acceleration_x = msg.acceleration.linear.x
                acceleration_y = msg.acceleration.linear.y
                acceleration_z = msg.acceleration.linear.z
                acceleration_magnitude = np.sqrt(acceleration_x**2 + acceleration_y**2 + acceleration_z**2)

                time_data.append(current_time)
                velocity_x_data.append(velocity_x)
                velocity_y_data.append(velocity_y)
                velocity_z_data.append(velocity_z)
                velocity_magnitude_data.append(velocity_magnitude)

                acceleration_x_data.append(acceleration_x)
                acceleration_y_data.append(acceleration_y)
                acceleration_z_data.append(acceleration_z)
                acceleration_magnitude_data.append(acceleration_magnitude)
    
    return (time_data, velocity_x_data, velocity_y_data, velocity_z_data, velocity_magnitude_data,
            acceleration_x_data, acceleration_y_data, acceleration_z_data, acceleration_magnitude_data)

def plot_data(time_data, velocity_x_data, velocity_y_data, velocity_z_data, velocity_magnitude_data,
              acceleration_x_data, acceleration_y_data, acceleration_z_data, acceleration_magnitude_data):
    fig, axs = plt.subplots(4, 1, figsize=(10, 20))

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

    # Plot acceleration components (x, y, z) on the same graph
    axs[2].plot(time_data, acceleration_x_data, label='Acceleration X', color='r')
    axs[2].plot(time_data, acceleration_y_data, label='Acceleration Y', color='g')
    axs[2].plot(time_data, acceleration_z_data, label='Acceleration Z', color='b')
    axs[2].set_xlabel('Time (s)')
    axs[2].set_ylabel('Acceleration (m/s²)')
    axs[2].set_title('Acceleration Components vs Time')
    axs[2].legend(loc='lower right')

    # Plot acceleration magnitude
    axs[3].plot(time_data, acceleration_magnitude_data, label='Acceleration Magnitude', color='b')
    axs[3].set_xlabel('Time (s)')
    axs[3].set_ylabel('Acceleration (m/s²)')
    axs[3].set_title('Acceleration Magnitude vs Time')
    axs[3].legend(loc='lower right')

    plt.tight_layout()
    plt.show()

def compute_averages(velocity_magnitude_data, acceleration_magnitude_data):
    average_velocity_magnitude = np.mean(velocity_magnitude_data)
    average_acceleration_magnitude = np.mean(acceleration_magnitude_data)
    return average_velocity_magnitude, average_acceleration_magnitude

def main():
    #bag_file = '/mnt/c/Users/joaof/Documents/data/school/one_drone/AEP/tmp_bags/tmp_bag_2024-08-15-21-55-17.bag'
    bag_file = '/home/joaomendes/motion_workspace/src/data/tmp_bags/tmp_bag_2024-09-01-21-50-06.bag'
    state_topic = '/uav1/estimation_manager/uav_state'
    clock_topic = '/clock'

    # Extract data
    (time_data, velocity_x_data, velocity_y_data, velocity_z_data, velocity_magnitude_data,
     acceleration_x_data, acceleration_y_data, acceleration_z_data, acceleration_magnitude_data) = extract_data(bag_file, state_topic, time_limit=2500)

    # Compute averages
    average_velocity_magnitude, average_acceleration_magnitude = compute_averages(velocity_magnitude_data, acceleration_magnitude_data)

    # Print the averages
    print(f"Average Velocity Magnitude: {average_velocity_magnitude:.2f} m/s")
    print(f"Average Acceleration Magnitude: {average_acceleration_magnitude:.2f} m/s^2")

    # Plot data
    plot_data(time_data, velocity_x_data, velocity_y_data, velocity_z_data, velocity_magnitude_data,
              acceleration_x_data, acceleration_y_data, acceleration_z_data, acceleration_magnitude_data)

if __name__ == "__main__":
    main()