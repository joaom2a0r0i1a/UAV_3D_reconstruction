#include <iostream>
#include <ros/ros.h>
#include <Eigen/Dense>
#include "motion_planning_python/gain_evaluator.h"

std::ostream& operator<<(std::ostream& os, const GainEvaluator& gain_evaluator) {
  const voxblox::CameraModel& cam_model = gain_evaluator.getCameraModel();
  voxblox::AlignedVector<voxblox::Point> plane_points;
  cam_model.getFarPlanePoints(&plane_points);
  //std::vector<Eigen::Vector3d> corners = cam_model.getFarPlanePoints(points);
  os << "Camera Model Parameters:" << std::endl;

  // Print corners_C_
  os << "Frustum Corners:" << std::endl;
  for (const auto& corner : plane_points) {
    os << "  (" << corner.x() << ", " << corner.y() << ", " << corner.z() << ")" << std::endl;
  }
  return os;
}

int main(int argc, char** argv) {
  // Initialize ROS node
  ros::init(argc, argv, "gain_evaluator_node");
  ros::NodeHandle nh;

  // Create an instance of GainEvaluator
  GainEvaluator gain_evaluator;

  // Set camera model parameters using setCameraModelParametersFoV function
  double horizontal_fov_degrees = 60.0;  // Example value, adjust as needed
  double vertical_fov_degrees = 40.0;    // Example value, adjust as needed
  double min_distance = 0.1;      // Example value, adjust as needed
  double max_distance = 10.0;     // Example value, adjust as needed

  // Convert degrees to radians
  double horizontal_fov = (horizontal_fov_degrees * M_PI) / 180.0;
  double vertical_fov = (vertical_fov_degrees * M_PI) / 180.0;

  voxblox::Transformation T_C_B;
  
  T_C_B.setIdentity(); // Initialize to identity matrix

  gain_evaluator.setCameraModelParametersFoV(horizontal_fov, vertical_fov, min_distance, max_distance);
  gain_evaluator.setCameraExtrinsics(T_C_B);

  std::cout << gain_evaluator << std::endl;
  // Do other processing or ROS operations here

  // Spin ROS
  ros::spin();

  return 0;
}
