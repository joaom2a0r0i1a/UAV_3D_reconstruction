#!/usr/bin/env python3
# Runs the real planners in the MRS simulator without mavros, bridging state and setpoints.
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import PositionTarget
from mrs_msgs.msg import UavState, ReferenceStamped


class MrsSimBridge:
    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "uav1/world_origin")
        self.pub_pose = rospy.Publisher("~pose_out", PoseStamped, queue_size=10)
        self.pub_vel = rospy.Publisher("~velocity_out", TwistStamped, queue_size=10)
        self.pub_ref = rospy.Publisher("~reference_out", ReferenceStamped, queue_size=10)
        self.last_key = None
        rospy.Subscriber("~uav_state_in", UavState, self.cb_state, queue_size=10)
        rospy.Subscriber("~setpoint_in", PositionTarget, self.cb_setpoint, queue_size=10)
        rospy.loginfo("[mrs_sim_bridge]: up (reference frame: %s)", self.frame_id)

    def cb_state(self, msg):
        out = PoseStamped()
        out.header = msg.header
        out.pose = msg.pose
        self.pub_pose.publish(out)
        vel = TwistStamped()
        vel.header = msg.header
        vel.twist = msg.velocity
        self.pub_vel.publish(vel)

    def cb_setpoint(self, msg):
        # The planner streams the active setpoint every tick (mavros style); the MRS tracker latches its goal, so forward only when the target changes.
        key = (round(msg.position.x, 3), round(msg.position.y, 3),
               round(msg.position.z, 3), round(msg.yaw, 3))
        if key == self.last_key:
            return
        self.last_key = key
        ref = ReferenceStamped()
        ref.header.stamp = rospy.Time.now()
        ref.header.frame_id = self.frame_id
        ref.reference.position.x = msg.position.x
        ref.reference.position.y = msg.position.y
        ref.reference.position.z = msg.position.z
        ref.reference.heading = msg.yaw
        self.pub_ref.publish(ref)
        rospy.loginfo("[mrs_sim_bridge]: reference -> [%.2f, %.2f, %.2f] hdg=%.2f",
                      msg.position.x, msg.position.y, msg.position.z, msg.yaw)


if __name__ == "__main__":
    rospy.init_node("mrs_sim_bridge")
    MrsSimBridge()
    rospy.spin()
