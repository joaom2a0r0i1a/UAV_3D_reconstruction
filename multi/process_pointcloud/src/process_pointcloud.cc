#include <ros/ros.h>
#include <process_pointcloud/process_pointcloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/filter.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <mrs_lib/param_loader.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>

#define SQ(x) ((x)*(x))

Processing::Processing(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private) : nh_(nh), nh_private_(nh_private) {
  /* Parameter loading */
  mrs_lib::ParamLoader param_loader(nh_private_, "processing");

  // Namespace
  param_loader.loadParam("uav_namespace", uav_namespace);
  param_loader.loadParam("max_distance", max_distance);

  std::vector<std::string> namespaces = {"uav1", "uav2", "uav3"};
  //std::vector<std::string> namespaces = {"uav1", "uav2"};
  namespaces.erase(std::remove(namespaces.begin(), namespaces.end(), uav_namespace), namespaces.end());
  uav_frames = {namespaces[0] + "/fcu", namespaces[1] + "/fcu"};
  //uav_frames = {namespaces[0] + "/rgbd_front_pitched/aligned_depth_to_color_optical"};
  
  sub_pointcloud = nh_.subscribe("pointcloud_in", 1, &Processing::process_pointcloud, this);
  pub_pointcloud = nh_.advertise<sensor_msgs::PointCloud2>("transformed_pointcloud_out", 5, true);
}

void Processing::process_pointcloud(const sensor_msgs::PointCloud2::ConstPtr& pointcloud) {
  // Prune Points near Other UAVs

  // Start timing
  auto start = std::chrono::high_resolution_clock::now();

  std::vector<tf::Vector3> agents;
  for (typename std::vector<std::string>::iterator it = uav_frames.begin();
       it != uav_frames.end(); it++) {
    tf::StampedTransform tf_transform;
    ros::Time time_to_lookup = pointcloud->header.stamp;
    if (!tf_listener.canTransform(pointcloud->header.frame_id, *it, time_to_lookup)) {
      time_to_lookup = ros::Time(0);
      ROS_WARN("Using latest TF transform instead of timestamp match.");
    }
    try {
      //tf_listener.waitForTransform(pointcloud->header.frame_id, *it, time_to_lookup, ros::Duration(3.0));
      tf_listener.lookupTransform(pointcloud->header.frame_id, *it, time_to_lookup, tf_transform);
    } catch (tf::TransformException& ex) {
      ROS_ERROR_STREAM("Error getting TF transform from sensor data: " << ex.what());
      return;
    }
    agents.push_back(tf_transform.getOrigin());
  }

  // Prepare pointcloud.
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*pointcloud, *cloud);
  // Remove NaN values, if any.
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
  // Remove Points near UAVs
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pruned(new pcl::PointCloud<pcl::PointXYZ>);
  for (pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud->begin(); it != cloud->end(); ++it) {
    bool insert = true;
    for (typename std::vector<tf::Vector3>::iterator itPose = agents.begin();
        itPose != agents.end(); itPose++) {
      if (SQ(it->x - itPose->x()) + SQ(it->y - itPose->y()) + SQ(it->z - itPose->z()) < max_distance) {
        insert = false;
      }
    }
    if (insert) {
      cloud_pruned->push_back(*it);
    }
  }
  // Publish pruned pointcloud
  sensor_msgs::PointCloud2::Ptr pointcloudOut(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*cloud_pruned, *pointcloudOut);
  pointcloudOut->header = pointcloud->header;
  pub_pointcloud.publish(pointcloudOut);

  // End timing
  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = end - start;
  ROS_INFO("Point cloud processing took: %.6f seconds", elapsed.count());
}
