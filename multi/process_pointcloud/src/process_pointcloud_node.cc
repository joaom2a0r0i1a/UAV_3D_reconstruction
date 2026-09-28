#include <ros/ros.h>
#include <process_pointcloud.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "processing");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  Processing processing(nh, nh_private);

  ros::spin();

  return 0;
}
