#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <cmath>

class SafeTfBroadcaster {
public:
  SafeTfBroadcaster() : has_prev_time_(false), max_time_jump_sec_(5.0) {
    ros::NodeHandle nh;
    pose_sub_ = nh.subscribe("/mavros/local_position/pose", 10, &SafeTfBroadcaster::poseCallback, this);
  }

  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    ros::Time current_time = msg->header.stamp;

    // Check timestamp jump
    if (has_prev_time_) {
      double dt = std::abs((current_time - prev_time_).toSec());
      if (dt > max_time_jump_sec_) {
        ROS_WARN_STREAM_THROTTLE(10, "[TF BROADCAST] Timestamp jump detected: " << dt << "s. Skipping this transform.");
        return;
      }
    }

    // Check quaternion validity
    const auto& q = msg->pose.orientation;
    double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (std::isnan(norm) || norm < 0.1 || norm > 1.1) {
      ROS_WARN_THROTTLE(5, "[TF BROADCAST] Invalid quaternion detected. Norm: %.3f. Skipping transform.", norm);
      return;
    }

    // Check position sanity (optionally define bounds)
    const auto& p = msg->pose.position;
    if (std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z) ||
        std::abs(p.x) > 1e4 || std::abs(p.y) > 1e4 || std::abs(p.z) > 1e4) {
      ROS_WARN_THROTTLE(5, "[TF BROADCAST] Invalid position values. Skipping transform.");
      return;
    }

    // All checks passed — broadcast the transform
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(p.x, p.y, p.z));
    tf::Quaternion tf_q(q.x, q.y, q.z, q.w);
    tf_q.normalize();
    transform.setRotation(tf_q);

    tf_broadcaster_.sendTransform(tf::StampedTransform(
        transform, current_time, "map", "base_link"));

    // Update previous time
    prev_time_ = current_time;
    has_prev_time_ = true;
  }

private:
  ros::Subscriber pose_sub_;
  tf::TransformBroadcaster tf_broadcaster_;
  ros::Time prev_time_;
  bool has_prev_time_;
  double max_time_jump_sec_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "safe_tf_broadcaster");
  SafeTfBroadcaster broadcaster;
  ros::spin();
  return 0;
}

