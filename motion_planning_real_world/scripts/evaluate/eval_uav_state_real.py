import rosbag
import matplotlib.pyplot as plt
from mrs_msgs.msg import UavState
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock
import numpy as np
import math
import os

# Function to extract data from rosbag
def extract_data(bag_file, state_topic, time_start=0, time_limit=2000):
    time_data = []
    velocity_x_data = []
    velocity_y_data = []
    velocity_z_data = []
    velocity_magnitude_data = []
    current_time = None
    count = 0
    max_value = 0

    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=[state_topic]):
            if topic == state_topic:

                if current_time is None:
                    first_time = t.to_sec()
                    current_time = t.to_sec()
                else:
                    current_time = t.to_sec() - first_time

                #print(current_time)

                if current_time < time_start:
                    continue

                # Limit time in case simulation finishes early
                #if current_time > time_limit:
                #    break

                velocity_x = msg.twist.linear.x
                velocity_y = msg.twist.linear.y
                velocity_z = msg.twist.linear.z
                velocity_magnitude = np.sqrt(velocity_x**2 + velocity_y**2 + velocity_z**2)
                if velocity_magnitude > 0.7 or math.isnan(velocity_magnitude):
                    continue
                elif velocity_magnitude > 0.5:
                    if velocity_magnitude > max_value:
                        max_value = velocity_magnitude
                    #print(velocity_magnitude)
                    count += 1

                #acceleration_x = msg.acceleration.linear.x
                #acceleration_y = msg.acceleration.linear.y
                #acceleration_z = msg.acceleration.linear.z
                #acceleration_magnitude = np.sqrt(acceleration_x**2 + acceleration_y**2 + acceleration_z**2)

                time_data.append(current_time)
                velocity_x_data.append(velocity_x)
                velocity_y_data.append(velocity_y)
                velocity_z_data.append(velocity_z)
                velocity_magnitude_data.append(velocity_magnitude)

                #acceleration_x_data.append(acceleration_x)
                #acceleration_y_data.append(acceleration_y)
                #acceleration_z_data.append(acceleration_z)
                #acceleration_magnitude_data.append(acceleration_magnitude)
    #print(count)
    #print(max_value)
    return (time_data, velocity_x_data, velocity_y_data, velocity_z_data, velocity_magnitude_data)

def extract_distance(bag_file, state_topic, time_limit=2000):
    time_data = []
    distance = 0.0
    previous_position = None

    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=[state_topic]):
            if topic == state_topic:
                current_time = t.to_sec()

                # Limit time in case simulation finishes early
                #if current_time > time_limit:
                #    break

                x = msg.pose.position.x
                y = msg.pose.position.y
                z = msg.pose.position.z
                offset = np.sqrt(x**2 + y**2 + z**2)
                if offset > 500:
                    continue
                current_position = np.array([x, y, z])

                if previous_position is not None:
                    dist = np.linalg.norm(current_position - previous_position)
                    distance += dist

                previous_position = current_position

    return (distance)

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

def compute_averages(velocity_magnitude_data):
    average_velocity_magnitude = np.mean(velocity_magnitude_data)
    #average_acceleration_magnitude = np.mean(acceleration_magnitude_data)
    return average_velocity_magnitude #, average_acceleration_magnitude

def main():
    #bag_file = '/mnt/c/Users/joaof/Documents/data/school/one_drone/Kinodynamic AEP (ours)/tmp_bags/tmp_bag_2024-09-04-12-02-33.bag'
    #bag_file = '/home/joaomendes/motion_workspace/src/data/tmp_bags/tmp_bag_2024-09-05-17-34-32.bag'
    #directory_bags = '/mnt/c/Users/joaof/Documents/data/police_station/one_drone'
    #state_topic = '/uav1/estimation_manager/uav_state'
    #bag_file = '/home/joaomendes/real_rosbags/experiments/aep/1_2025_06_21_11_53_20/_2025-06-21-11-54-18.bag'
    directory_bags = '/home/joaomendes/real_rosbags/experiments/kaep'
    state_topic = '/mavros/local_position/velocity_local'
    pose_topic = '/mavros/local_position/pose'
    #start_topic = '/uav2/simulation_ready'
    #state_topic = ['/uav1/estimation_manager/uav_state', '/uav2/estimation_manager/uav_state', '/uav3/estimation_manager/uav_state']
    clock_topic = '/clock'

    #(time_data, velocity_x_data, velocity_y_data, velocity_z_data, velocity_magnitude_data,
    #acceleration_x_data, acceleration_y_data, acceleration_z_data, acceleration_magnitude_data) = extract_data(bag_file, state_topic, time_limit=1000)

    #average_velocity_magnitude, average_acceleration_magnitude = compute_averages(velocity_magnitude_data, acceleration_magnitude_data)
    total_average_velocity = []
    average_distance = []
    for directory in os.listdir(directory_bags):
        directory_aux = os.path.join(directory_bags, directory)
        #directory_aux = os.path.join(directory_bags, directory, "Connected", "tmp_bags")
        for filename in os.listdir(directory_aux):
            if filename == "tmux":
                continue
            # Extract data
            time_start=0
            filename_aux = os.path.join(directory_aux, filename)
            (time_data, velocity_x_data, velocity_y_data, velocity_z_data, velocity_magnitude_data) = extract_data(filename_aux, state_topic, time_start, time_limit=900)

            distance = 0.0
            #for state in state_topic:
            #    distance += extract_distance(filename_aux, state, time_limit=1200)
            distance = extract_distance(filename_aux, pose_topic, time_limit=1800)

            # Compute averages
            average_velocity_magnitude = compute_averages(velocity_magnitude_data)
            total_average_velocity.append(average_velocity_magnitude)
            average_distance.append(distance)
            print(f"{directory}: Average Velocity: {average_velocity_magnitude:.3f}")
            print(f"{directory}: Distance Travelled: {distance:.3f}")

    #KNBV
    # 21, 14, 40
    #KAEP
    # 27, 31, 21
    #NBV
    # 35, 20, 23
    #AEP
    # 39, 24, 17
    velocity_mean = np.mean(total_average_velocity)
    distance_mean = np.mean(average_distance)

    velocity_variance = sum((x - velocity_mean) ** 2 for x in total_average_velocity) / len(total_average_velocity)
    velocity_standard_deviation = np.sqrt(velocity_variance)
        
    distance_variance = sum((x - distance_mean) ** 2 for x in average_distance) / len(average_distance)
    distance_standard_deviation = np.sqrt(distance_variance)

    # Print the averages
    #print(f"Average Velocity Magnitude: {average_velocity_magnitude:.2f} m/s")
    #print(f"Average Acceleration Magnitude: {average_acceleration_magnitude:.2f} m/s^2")

    print(f"Mean Velocity Magnitude: {velocity_mean:.3f} +/- {velocity_standard_deviation:.3f} m/s")
    print(f"Mean Distance Travelled: {distance_mean:.3f} +/- {distance_standard_deviation:.3f} m")

    # Plot data
    #print(f"Average Velocity Magnitude: {average_velocity_magnitude:.2f} m/s")
    #print(f"Average Acceleration Magnitude: {average_acceleration_magnitude:.2f} m/s^2")

    #plot_data(time_data, velocity_x_data, velocity_y_data, velocity_z_data, velocity_magnitude_data,
    #          acceleration_x_data, acceleration_y_data, acceleration_z_data, acceleration_magnitude_data)

if __name__ == "__main__":
    main()