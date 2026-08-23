#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>

class Processing {
 public:
  Processing(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  void process_pointcloud(const sensor_msgs::PointCloud2::ConstPtr& pointcloud);
 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  std::string uav_namespace;
  double max_distance;
  std::vector<std::string> uav_frames;
  tf::TransformListener tf_listener;
  
  ros::Publisher pub_pointcloud;
  ros::Subscriber sub_pointcloud;
};
