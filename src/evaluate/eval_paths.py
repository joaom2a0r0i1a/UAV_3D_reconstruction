import rospy
import rosbag
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from rosgraph_msgs.msg import Clock

def extract_path(bag_file, pose_topic, clock_topic):
    path = Path()
    path.header.frame_id = "uav1/world_origin"

    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=[pose_topic, clock_topic]):
            if topic == clock_topic:
                # Extract time from /clock topic
                current_time = msg.clock.secs + msg.clock.nsecs * 1e-9
            elif topic == pose_topic:
                # Extract pose from the specified topic
                pose_stamped = PoseStamped()
                pose_stamped.header.stamp = rospy.Time.from_sec(current_time)
                pose_stamped.header.frame_id = "uav1/world_origin"
                pose_stamped.pose = msg.pose

                path.poses.append(pose_stamped)

    return path

def extract_reference(bag_file, pose_topic, clock_topic):
    path = Path()
    path.header.frame_id = "uav1/world_origin"

    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=[pose_topic, clock_topic]):
            if topic == clock_topic:
                # Extract time from /clock topic
                current_time = msg.clock.secs + msg.clock.nsecs * 1e-9
            elif topic == pose_topic:
                # Extract pose from the specified topic
                pose_stamped = PoseStamped()
                pose_stamped.header.stamp = rospy.Time.from_sec(current_time)
                pose_stamped.header.frame_id = "uav1/world_origin"
                pose_stamped.pose.position = msg.position

                path.poses.append(pose_stamped)

    return path

def publish_path():
    rospy.init_node('path_publisher', anonymous=True)
    path_pub = rospy.Publisher('/uav1/finished_path', Path, queue_size=10)
    reference_pub = rospy.Publisher('/uav1/reference_path', Path, queue_size=10)

    bag_file = '/home/joaomendes/workspace1/src/motion_planning_python/data/tmp_bags/tmp_bag_2024-05-23-10-53-33.bag'
    uav_pose_topic = '/uav1/estimation_manager/uav_state'
    uav_reference_topic = '/uav1/reference_out'
    clock_topic = '/clock'

    path = extract_path(bag_file, uav_pose_topic, clock_topic)
    print("HALFWAY THERE")
    reference = extract_reference(bag_file, uav_reference_topic, clock_topic)
    print("FINITO")

    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        path.header.stamp = rospy.Time.now()
        reference.header.stamp = rospy.Time.now()
        path_pub.publish(path)
        reference_pub.publish(reference)
        rate.sleep()

if __name__ == "__main__":
    try:
        publish_path()
    except rospy.ROSInterruptException:
        pass
