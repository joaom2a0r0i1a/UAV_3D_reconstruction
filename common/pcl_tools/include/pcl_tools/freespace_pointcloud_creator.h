#pragma once

/* includes and typedefs //{ */
#include <mutex>
#include <tuple>
#include <set>

// basic ros
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/filter_indices.h>

#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/param_loader.h>

#include <sensor_msgs/Image.h>
#include <tf/transform_listener.h>

//}

namespace pcl_tools
{

  /* class FreespacePointcloudCreator //{ */
  class FreespacePointcloudCreator : public nodelet::Nodelet
  {

    public:
      virtual void onInit();

    private:
      ros::NodeHandle m_nh;

      mrs_lib::SubscribeHandler<sensor_msgs::Image> m_sub_depth_image_in;
      ros::Publisher m_pub_freespace_pc_out;
      tf::TransformListener listener;

      bool is_in_box(const double x, const double y, const double z);
      void depth_image_callback(const sensor_msgs::Image::ConstPtr depth_image);

      double m_sensor_hfov, m_sensor_vfov, m_sensor_max_range;
      bool m_row_major;
      double min_x_, min_y_, min_z_, max_x_, max_y_, max_z_;
  };
  //}

}  // namespace pcl_tools
