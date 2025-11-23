import rosbag
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter

def extract_positions_from_mavros(bag_path, topic='/mavros/local_position/pose', pos_limit=100, duration_limit=300):
    """
    Extracts x, y, z positions and timestamps from MAVROS local position topic.

    Returns:
    - x, y, z: lists of position components
    - t: list of timestamps (in seconds)
    """
    x, y, z, time_stamps = [], [], [], []
    start_time = None

    with rosbag.Bag(bag_path, 'r') as bag:
        for topic_, msg, t in bag.read_messages(topics=[topic]):
            t_sec = t.to_sec()

            if start_time is None:
                start_time = t_sec

            # Reject samples with unrealistic acceleration spikes
            if abs(msg.pose.position.x) > pos_limit or abs(msg.pose.position.y) > pos_limit or abs(msg.pose.position.z) > pos_limit:
                continue

            # Stop reading if duration exceeds the limit
            if (t_sec - start_time) > duration_limit:
                break

            x.append(msg.pose.position.x)
            y.append(msg.pose.position.y)
            z.append(msg.pose.position.z)
            time_stamps.append(t_sec)

    return np.array(x), np.array(y), np.array(z), np.array(time_stamps)


def compute_curvature(x, y, z, t, smooth=False, window=11, polyorder=3):
    """
    Computes curvature along a 3D trajectory.

    Parameters:
    - x, y, z, t: position and time arrays
    - smooth: whether to apply Savitzky-Golay smoothing
    - window, polyorder: smoothing params

    Returns:
    - curvature: curvature at each time step
    - avg_curvature: mean curvature over the trajectory
    """
    if smooth:
        x = savgol_filter(x, window, polyorder)
        y = savgol_filter(y, window, polyorder)
        z = savgol_filter(z, window, polyorder)

    # First derivatives (velocity)
    dx = np.gradient(x, t)
    dy = np.gradient(y, t)
    dz = np.gradient(z, t)

    # Second derivatives (acceleration)
    ddx = np.gradient(dx, t)
    ddy = np.gradient(dy, t)
    ddz = np.gradient(dz, t)

    # Cross product of velocity and acceleration
    v = np.stack((dx, dy, dz), axis=1)
    a = np.stack((ddx, ddy, ddz), axis=1)
    cross = np.cross(v, a)

    # Curvature = ||v × a|| / ||v||^3
    num = np.linalg.norm(cross, axis=1)
    denom = np.linalg.norm(v, axis=1)**3

    with np.errstate(divide='ignore', invalid='ignore'):
        curvature = np.where(denom != 0, num / denom, 0)

    avg_curvature = np.mean(curvature[np.isfinite(curvature)])
    return curvature, avg_curvature


def main():
    bag_path = "/home/joaomendes/real_rosbags/experiments/kaep/12_2025_06_28_11_50_28/_2025-06-28-11-51-18.bag"  # Replace this with your actual bag file
    x, y, z, t = extract_positions_from_mavros(bag_path)

    curvature, avg_curv = compute_curvature(x, y, z, t)

    print(f"Average Trajectory Curvature: {avg_curv:.5f} [1/m]")

    # Optional: plot curvature over time
    plt.plot(t, curvature)
    plt.xlabel("Time [s]")
    plt.ylabel("Curvature [1/m]")
    plt.title("Trajectory Curvature Over Time")
    plt.grid(True)
    plt.show()


if __name__ == "__main__":
    main()
