import rosbag
import numpy as np
from sensor_msgs.msg import Imu
import os

def extract_accel_from_bag_mrs(bag_path, topic='/estimation_manager/uav_state', acc_limit=10.0, duration_limit=2100.0):
    """
    Extract linear acceleration (x, y, z) and timestamps from a ROS bag file.
    
    Returns:
    - acc_x, acc_y, acc_z: list of acceleration components
    - time_stamps: list of timestamps (float, in seconds)
    """
    acc_x, acc_y, acc_z, time_stamps = [], [], [], []
    start_time = None

    with rosbag.Bag(bag_path, 'r') as bag:
        for topic_, msg, t in bag.read_messages(topics=[topic]):
            t_sec = t.to_sec()

            accx = msg.acceleration.linear.x
            accy = msg.acceleration.linear.y
            accz = msg.acceleration.linear.z

            if t_sec > duration_limit:
                break

            # Reject samples with unrealistic acceleration spikes
            if abs(accx) > acc_limit or abs(accy) > acc_limit or abs(accz) > acc_limit:
                continue

            acc_x.append(accx)
            acc_y.append(accy)
            acc_z.append(accz)
            time_stamps.append(t_sec)

    return acc_x, acc_y, acc_z, time_stamps

def extract_accel_from_bag(bag_path, topic='/imu/data', acc_limit=10.0, duration_limit=2100.0):
    """
    Extract linear acceleration (x, y, z) and timestamps from a ROS bag file.
    
    Returns:
    - acc_x, acc_y, acc_z: list of acceleration components
    - time_stamps: list of timestamps (float, in seconds)
    """
    acc_x, acc_y, acc_z, time_stamps = [], [], [], []
    start_time = None

    with rosbag.Bag(bag_path, 'r') as bag:
        for topic_, msg, t in bag.read_messages(topics=[topic]):
            t_sec = t.to_sec()

            if start_time is None:
                start_time = t_sec

            acc = msg.linear_acceleration

            # Reject samples with unrealistic acceleration spikes
            if abs(acc.x) > acc_limit or abs(acc.y) > acc_limit or abs(acc.z) > acc_limit:
                continue

            # Stop reading if duration exceeds the limit
            if (t_sec - start_time) > duration_limit:
                break

            acc_x.append(acc.x)
            acc_y.append(acc.y)
            acc_z.append(acc.z)
            time_stamps.append(t_sec)

    return acc_x, acc_y, acc_z, time_stamps


def compute_total_jerk_cost(ax, ay, az, time_stamps):
    """
    Compute jerk cost from acceleration and timestamps.
    """
    ax = np.array(ax)
    ay = np.array(ay)
    az = np.array(az)
    time_stamps = np.array(time_stamps)

    if len(time_stamps) < 3:
        return 0.0, 0.0, 0.0  # Not enough data to compute jerk

    # Compute first time difference
    dt = np.diff(time_stamps)

    # Identify valid (non-zero) dt entries
    valid = dt > 1e-6  # 1 microsecond tolerance

    # Only keep valid entries for jerk computation
    ax_diff = np.diff(ax)[valid]
    ay_diff = np.diff(ay)[valid]
    az_diff = np.diff(az)[valid]
    dt_valid = dt[valid]

    # Avoid divide-by-zero
    if len(dt_valid) == 0:
        return 0.0, 0.0, 0.0

    jx = ax_diff / dt_valid
    jy = ay_diff / dt_valid
    jz = az_diff / dt_valid

    jerk_mag = np.sqrt(jx**2 + jy**2 + jz**2)
    # Safe total jerk cost
    total_jerk_cost = np.sum(jerk_mag**2 * dt_valid) / time_stamps[-1]

    # Average acceleration in XY plane (using original data)
    accel_xy_mag = np.sqrt(ax**2 + ay**2)
    avg_accel_xy = np.mean(accel_xy_mag)

    # Average vertical (Z-axis) acceleration
    avg_accel_z = np.mean(np.abs(az))  # use abs if you care about magnitude only

    return total_jerk_cost, avg_accel_xy, avg_accel_z

# === MAIN ===
if __name__ == "__main__":
    #bag_path = "/home/joaomendes/real_rosbags/experiments/kaep/12_2025_06_28_11_50_28/_2025-06-28-11-51-18.bag"
    #bag_path = "/home/joaomendes/motion_workspace/src/data/tmp_bags/tmp_bag_2025-07-07-19-17-12.bag"
    #bag_path = "/mnt/c/Users/joaof/Documents/data/school/one_drone/RH-NBVP/tmp_bags/tmp_bag_2024-07-11-12-32-57.bag"
    #bag_path = "/mnt/c/Users/joaof/Documents/data/school/one_drone/KinodynamicRH-NBVP/tmp_bags/tmp_bag_2024-09-02-18-44-49.bag"
    #bag_path = "/mnt/c/Users/joaof/Documents/data/school/one_drone/KinodynamicAEP/tmp_bags/tmp_bag_2024-09-03-19-48-40.bag"
    #imu_topic = "/mavros/imu/data"  # change as needed
    directory_bags = "/mnt/c/Users/joaof/Documents/data/school/one_drone"
    imu_topic = "/uav1/estimation_manager/uav_state"

    for directory in os.listdir(directory_bags):
        directory_aux = os.path.join(directory_bags, directory, "tmp_bags")
        #directory_aux = os.path.join(directory_bags, directory, "Connected", "tmp_bags")
        total_average_jerk = []
        average_acceleration_xy = []
        average_acceleration_z = []
        for filename in os.listdir(directory_aux):
            filename_aux = os.path.join(directory_aux, filename)
            #ax, ay, az, times = extract_accel_from_bag(bag_path, topic=imu_topic)
            ax, ay, az, times = extract_accel_from_bag_mrs(filename_aux, topic=imu_topic)
            
            jerk_cost, avg_xy, avg_z = compute_total_jerk_cost(ax, ay, az, times)
            print(f"{directory}: Jerk cost: {jerk_cost:.2f} m²/s⁵")
            print(f"{directory}:Average acceleration (XY): {avg_xy:.3f} m/s²")
            print(f"{directory}:Average acceleration (Z): {avg_z:.3f} m/s²")
            
            total_average_jerk.append(jerk_cost)
            average_acceleration_xy.append(avg_xy)
            average_acceleration_z.append(avg_z)
        
        total_jerk_mean = np.mean(total_average_jerk)
        accel_xy_mean = np.mean(average_acceleration_xy)
        accel_z_mean = np.mean(average_acceleration_z)
        
        total_jerk_variance = sum((x - total_jerk_mean) ** 2 for x in total_average_jerk) / len(total_average_jerk)
        total_jerk_standard_deviation = np.sqrt(total_jerk_variance)
        
        accel_xy_variance = sum((x - accel_xy_mean) ** 2 for x in average_acceleration_xy) / len(average_acceleration_xy)
        accel_xy_standard_deviation = np.sqrt(accel_xy_variance)

        accel_z_variance = sum((x - accel_z_mean) ** 2 for x in average_acceleration_z) / len(average_acceleration_z)
        accel_z_standard_deviation = np.sqrt(accel_z_variance)

        print(f"{directory}:Mean Total Jerk: {total_jerk_mean:.3f} +/- {total_jerk_variance:.3f} m/s")
        print(f"{directory}:Mean Accel XY Magnitude: {accel_xy_mean:.3f} +/- {accel_xy_standard_deviation:.3f} m/s")
        print(f"{directory}:Mean Accel Z Magnitude: {accel_z_mean:.3f} +/- {accel_z_standard_deviation:.3f} m/s")
