import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from mavros_msgs.msg import PositionTarget
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

class DroneTrajectory:
    def __init__(self):
        rospy.init_node('drone_trajectory_node', anonymous=True)
        
        self.local_position = None
        self.initial_position = None
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.local_position_callback)
        self.setpoint_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        self.marker_pub = rospy.Publisher('/trajectory_markers', MarkerArray, queue_size=10)
        self.current_position_pub = rospy.Publisher('/current_position_markers', Marker, queue_size=10)
        
        self.rate = rospy.Rate(10)  # 10Hz update rate
        self.max_acceleration = 1.0  # Maximum acceleration
        self.max_velocity = 1.0  # Maximum positional velocity
        self.max_yaw_velocity = 1.0  # Maximum yaw velocity (rad/s)
        self.max_yaw_acceleration = 1.0  # Maximum yaw acceleration (rad/s^2)
        
        self.previous_x = 0.0
        self.previous_y = 0.0
        self.previous_z = 0.0
        self.previous_vx = 0.0
        self.previous_vy = 0.0
        self.previous_vz = 0.0
        self.previous_yaw = 0.0
        self.previous_yaw_velocity = 0.0
        self.target_yaw = np.random.uniform(-np.pi, np.pi)  # Random target yaw for the trajectory
        
        self.current_positions = []
        self.log_file = open("trajectory_log.txt", "w")
        
    
    def local_position_callback(self, msg):
        self.local_position = msg.pose.position
        #if len(self.current_positions) > 10:
        #    self.current_positions.pop(0)
        if self.initial_position is None:  # Set initial position only once
            self.initial_position = (self.local_position.x, self.local_position.y, self.local_position.z)
        self.current_positions.append((self.local_position.x, self.local_position.y, self.local_position.z))
        self.publish_current_position()

    def sample_acceleration(self):
        """Sample acceleration excluding values in (-max_accel/2, max_accel/2)"""
        def sample_excluding_range(min_val, max_val, exclusion_range):
            while True:
                value = np.random.uniform(min_val, max_val)
                if abs(value) > exclusion_range:
                    return value

        exclusion_threshold = self.max_acceleration / 2
        ax = sample_excluding_range(-self.max_acceleration, self.max_acceleration, exclusion_threshold)
        ay = sample_excluding_range(-self.max_acceleration, self.max_acceleration, exclusion_threshold)
        
        if self.local_position.z > 10:
            az = sample_excluding_range(-self.max_acceleration, self.max_acceleration, exclusion_threshold)
        else:
            az = sample_excluding_range(0, self.max_acceleration, exclusion_threshold)

        return ax, ay, az


    def calculate_motion(self, t, ax, ay, az):
        """Compute position, velocity, and yaw using Newton's equations of motion"""
        dt = 0.1  # Time step
        
        x = self.previous_x + self.previous_vx * dt + 0.5 * ax * dt**2
        y = self.previous_y + self.previous_vy * dt + 0.5 * ay * dt**2
        z = self.previous_z + self.previous_vz * dt + 0.5 * az * dt**2
        
        vx = np.clip(self.previous_vx + ax * dt, -self.max_velocity, self.max_velocity)
        vy = np.clip(self.previous_vy + ay * dt, -self.max_velocity, self.max_velocity)
        vz = np.clip(self.previous_vz + az * dt, -self.max_velocity, self.max_velocity)

        rospy.loginfo("Velocity: ({}, {}, {})".format(vx, vy, vz))
        
        yaw_error = self.target_yaw - self.previous_yaw
        if yaw_error < 0.4:
            yaw_acceleration = 0
        else:
            yaw_acceleration = np.sign(yaw_error) * self.max_yaw_acceleration
        yaw_velocity = np.clip(self.previous_yaw_velocity + yaw_acceleration * dt, -self.max_yaw_velocity, self.max_yaw_velocity)
        yaw = self.previous_yaw + yaw_velocity * dt
        
        self.previous_x, self.previous_y, self.previous_z = x, y, z
        self.previous_vx, self.previous_vy, self.previous_vz = vx, vy, vz
        self.previous_yaw = yaw
        self.previous_yaw_velocity = yaw_velocity
        
        return x, y, z, vx, vy, vz, yaw

    def publish_markers(self, waypoints):
        marker_array = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "waypoints"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.2  # Line width
        marker.color = ColorRGBA(0.0, 1.0, 0.0, 1.0)
        marker.lifetime = rospy.Duration(50)
        
        for i in range(len(waypoints) - 1):
            marker.points.append(Point(*waypoints[i]))
            marker.points.append(Point(*waypoints[i + 1]))
        
        marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)
    
    def publish_current_position(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "current_position"
        marker.id = 1
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.2  # Line width
        marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)  # Red for current path
        marker.lifetime = rospy.Duration(50)
        
        for i in range(len(self.current_positions) - 1):
            marker.points.append(Point(*self.current_positions[i]))
            marker.points.append(Point(*self.current_positions[i + 1]))
        
        self.current_position_pub.publish(marker)

    def send_trajectory(self):
        if self.local_position is None:
            rospy.logwarn("Waiting for local position data...")
            rospy.sleep(1)
            return
        
        waypoints = []
        self.previous_x, self.previous_y, self.previous_z = self.initial_position[0], self.initial_position[1], self.initial_position[2]
        
        for _ in range(8):  # Number of different trajectories
            t = 0.0
            ax, ay, az = self.sample_acceleration()
            rospy.loginfo("Starting new trajectory with acceleration: ({}, {}, {})".format(ax, ay, az))
            
            trajectory_points = []  # Store trajectory points
            while t < 3:
                x, y, z, vx, vy, vz, yaw = self.calculate_motion(t, ax, ay, az)
                trajectory_points.append((x, y, z, vx, vy, vz, yaw))
                t += 0.1

            # Send trajectory excluding the last 6 points
            for i in range(len(trajectory_points)):  
                x, y, z, vx, vy, vz, yaw = trajectory_points[i]
                
                setpoint = PositionTarget()
                setpoint.header = Header()
                setpoint.header.stamp = rospy.Time.now()
                setpoint.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
                setpoint.type_mask = PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ | \
                            PositionTarget.IGNORE_YAW_RATE
                
                setpoint.position.x = x
                setpoint.position.y = y
                setpoint.position.z = z
                setpoint.velocity.x = vx
                setpoint.velocity.y = vy
                setpoint.velocity.z = vz
                #setpoint.acceleration_or_force.x = ax
                #setpoint.acceleration_or_force.y = ay
                #setpoint.acceleration_or_force.z = az
                setpoint.yaw = yaw
                
                self.setpoint_pub.publish(setpoint)
                waypoints.append((x, y, z))
                self.publish_markers(waypoints)
                self.log_file.write(f"{rospy.Time.now().to_sec()}, {x}, {y}, {z}, {self.local_position.x}, {self.local_position.y}, {self.local_position.z}\n")
                
                self.rate.sleep()

            #rospy.sleep(0.3)
            
            rospy.loginfo("Completed trajectory (excluding last 5 points)")
            self.target_yaw = np.random.uniform(-np.pi, np.pi)

        setpoint = PositionTarget()
        setpoint.header = Header()
        setpoint.header.stamp = rospy.Time.now()
        setpoint.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        setpoint.type_mask = PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ | PositionTarget.IGNORE_YAW_RATE
        
        setpoint.velocity.x = 0
        setpoint.velocity.y = 0
        setpoint.velocity.z = 0
        setpoint.acceleration_or_force.x = 0
        setpoint.acceleration_or_force.y = 0
        setpoint.acceleration_or_force.z = 0
        setpoint.yaw = yaw
        self.setpoint_pub.publish(setpoint)

if __name__ == '__main__':
    try:
        drone = DroneTrajectory()
        rospy.sleep(2)
        drone.send_trajectory()
    except rospy.ROSInterruptException:
        pass
