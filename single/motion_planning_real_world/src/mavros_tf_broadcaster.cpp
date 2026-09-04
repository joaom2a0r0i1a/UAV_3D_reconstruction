#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <cmath>

class SafeTfBroadcaster {
public:
  SafeTfBroadcaster() : has_prev_pos_(false), has_prev_time_(false) {
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    // 1e4 was too loose; a 1524 m pose in the 2025 aep2 flight passed it into the map.
    nh_private.param("max_position", max_position_, 500.0);
    nh_private.param("max_speed", max_speed_, 20.0);
    nh_private.param("max_time_jump", max_time_jump_sec_, 20.0);
    pose_sub_ = nh.subscribe("/mavros/local_position/pose", 10, &SafeTfBroadcaster::poseCallback, this);
  }

  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    const ros::Time current_time = msg->header.stamp;
    const auto& q = msg->pose.orientation;
    const auto& p = msg->pose.position;

    // Check quaternion validity
    const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (std::isnan(norm) || norm < 0.1 || norm > 1.1) {
      ROS_WARN_THROTTLE(5, "[TF BROADCAST] Invalid quaternion detected. Norm: %.3f. Skipping transform.", norm);
      return;
    }

    // Check position sanity
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
        std::abs(p.x) > max_position_ || std::abs(p.y) > max_position_ ||
        std::abs(p.z) > max_position_) {
      ROS_WARN_THROTTLE(5, "[TF BROADCAST] Implausible position [%.3g, %.3g, %.3g]. Skipping transform.",
                        p.x, p.y, p.z);
      return;
    }

    // Check timestamp jump
    if (has_prev_time_) {
      const double dt = std::abs((current_time - prev_time_).toSec());
      if (dt > max_time_jump_sec_) {
        ROS_WARN_THROTTLE(10, "[TF BROADCAST] Timestamp jump of %.2f s. Skipping this transform.", dt);
        return;
      }
    }

    // Check for jumps no drone could fly
    if (has_prev_pos_ && has_prev_time_) {
      const double dt = std::abs((current_time - prev_time_).toSec());
      const double jump = std::sqrt(std::pow(p.x - prev_pos_[0], 2) +
                                    std::pow(p.y - prev_pos_[1], 2) +
                                    std::pow(p.z - prev_pos_[2], 2));
      if (dt > 1e-3 && jump / dt > max_speed_) {
        ROS_WARN_THROTTLE(5, "[TF BROADCAST] Position jumped %.2f m in %.3f s. Skipping transform.",
                          jump, dt);
        return;
      }
    }

    broadcast(current_time, p, q);
    resync(current_time, p);
  }

  void broadcast(const ros::Time& stamp, const geometry_msgs::Point& p,
                 const geometry_msgs::Quaternion& q) {
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(p.x, p.y, p.z));
    tf::Quaternion tf_q(q.x, q.y, q.z, q.w);
    tf_q.normalize();
    transform.setRotation(tf_q);
    tf_broadcaster_.sendTransform(tf::StampedTransform(transform, stamp, "map", "base_link"));
  }

  void resync(const ros::Time& stamp, const geometry_msgs::Point& p) {
    prev_time_ = stamp;                 has_prev_time_ = true;
    prev_pos_[0] = p.x; prev_pos_[1] = p.y; prev_pos_[2] = p.z;
    has_prev_pos_ = true;
  }

private:
  ros::Subscriber pose_sub_;
  tf::TransformBroadcaster tf_broadcaster_;
  ros::Time prev_time_;
  double max_position_;
  double max_speed_;
  double prev_pos_[3];
  bool has_prev_pos_;
  bool has_prev_time_;
  double max_time_jump_sec_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "safe_tf_broadcaster");
  SafeTfBroadcaster broadcaster;
  ros::spin();
  return 0;
}

