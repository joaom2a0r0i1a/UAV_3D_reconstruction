#include <pcl_tools/freespace_pointcloud_creator.h>

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/filter_indices.h>
#include <opencv2/imgproc/imgproc.hpp>

namespace pcl_tools
{

  /* onInit() //{ */

  void FreespacePointcloudCreator::onInit()
  {
    m_nh = nodelet::Nodelet::getMTPrivateNodeHandle();

    NODELET_INFO("[FreespacePointcloudCreator]: Waiting for valid time...");
    ros::Time::waitForValid();

    // Get parameters from config file
    mrs_lib::ParamLoader param_loader(m_nh, "FreespacePointcloudCreator");

    param_loader.loadParam("sensor/hfov", m_sensor_hfov);
    param_loader.loadParam("sensor/vfov", m_sensor_vfov);
    param_loader.loadParam("sensor/max_range", m_sensor_max_range);
    param_loader.loadParam("data_row_major", m_row_major);

    param_loader.loadParam("box/min_x", min_x_);
    param_loader.loadParam("box/max_x", max_x_);
    param_loader.loadParam("box/min_y", min_y_);
    param_loader.loadParam("box/max_y", max_y_);
    param_loader.loadParam("box/min_z", min_z_);
    param_loader.loadParam("box/max_z", max_z_);

    if (!param_loader.loadedSuccessfully()) {
      NODELET_ERROR("[FreespacePointcloudCreator]: Some compulsory parameters were not loaded successfully, ending the node");
      ros::shutdown();
      return;
    }

    mrs_lib::SubscribeHandlerOptions shopts(m_nh);
    shopts.no_message_timeout = ros::Duration(5.0);
    mrs_lib::construct_object(m_sub_depth_image_in, shopts, "depth_image_in", &FreespacePointcloudCreator::depth_image_callback, this);
    m_pub_freespace_pc_out = m_nh.advertise<sensor_msgs::PointCloud2>("freespace_pointcloud_out", 1);
  }
  //}

  /* is_in_box() //{ */

  bool FreespacePointcloudCreator::is_in_box(const double x, const double y, const double z)
  {
    if (x > min_x_ && x < max_x_ && y > min_y_ && y < max_y_ && z > min_z_ && z < max_z_) {
      return true;
    } else {
      return false;
    }
  }

  /* depth_image_callback() //{ */

  void FreespacePointcloudCreator::depth_image_callback(const sensor_msgs::Image::ConstPtr depth_image)
  {
    // Convert sensor_msgs/Image to OpenCV format
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_32FC1);
    } catch (cv_bridge::Exception& e) {
      NODELET_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat& img = cv_ptr->image;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    cloud->width = img.cols;
    cloud->height = img.rows;
    cloud->is_dense = false;
    cloud->points.resize(cloud->width * cloud->height);

    const double yaw_halfrange = m_sensor_hfov / 2;
    const double yaw_step = m_sensor_hfov / (img.cols - 1);
    const double pitch_halfrange = m_sensor_vfov / 2;
    const double pitch_step = m_sensor_vfov / (img.rows - 1);
    const double camera_pitch = 10 / 180 * M_PI;
    size_t it = 0;

    tf::StampedTransform transform;
    try {
        listener.lookupTransform("uav1/world_origin", "uav1/rgbd_front_pitched/aligned_depth_to_color", ros::Time(0), transform);
    } catch (tf::TransformException& ex) {
        ROS_ERROR("%s", ex.what());
        return;
    }

    for (int row = 0; row < img.rows; ++row) {
      for (int col = 0; col < img.cols; ++col, ++it) {
        float depth = img.at<float>(row, col);

        //const double yaw = yaw_halfrange - yaw_step * (double(col) - img.cols / 2.0);
        //const double pitch = pitch_halfrange - pitch_step * (double(row) - img.rows / 2.0);
        const double yaw = -yaw_halfrange + yaw_step * double(col);
        const double pitch = pitch_halfrange - camera_pitch - pitch_step * double(row);

        pcl::PointXYZ& pt = cloud->points[it];

        double x_camera = m_sensor_max_range * std::cos(yaw) * std::cos(pitch);
        double y_camera = m_sensor_max_range * std::sin(yaw) * std::cos(pitch);
        double z_camera = m_sensor_max_range * std::sin(pitch);
        tf::Vector3 point_camera(x_camera, y_camera, z_camera);

        tf::Vector3 point_world = transform * point_camera;
        double x_world = point_world.x();
        double y_world = point_world.y();
        double z_world = point_world.z();

        if (depth == 0.0f && !is_in_box(x_world,y_world,z_world)) {// || std::isnan(depth)) {
          // Convert (yaw, pitch, depth) to (x, y, z)
          pt.x = x_camera;
          pt.y = y_camera;
          pt.z = z_camera;
        } else {
          // Set to zero if depth value exists (i.e., valid obstacle point)
          pt.x = pt.y = pt.z = 0.0;
        }
      }
    }

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);
    output.header = depth_image->header;
    output.header.frame_id = "uav1/rgbd_front_pitched/aligned_depth_to_color";

    // Publish the freespace point cloud
    m_pub_freespace_pc_out.publish(output);
  }
  //}

}  // namespace pcl_tools

PLUGINLIB_EXPORT_CLASS(pcl_tools::FreespacePointcloudCreator, nodelet::Nodelet);
