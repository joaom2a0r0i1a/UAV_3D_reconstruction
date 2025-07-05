#include <ros/ros.h>
#include <eth_trajectory_generation/timing.h>
#include <voxblox/integrator/integrator_utils.h>

#include "rrt_construction/gain_evaluator.h"

GainEvaluator::GainEvaluator(const ros::NodeHandle& nh_private) {
  /*// IST Lamp
  min_x_ = 2.0;
  max_x_ = 7.0;
  min_y_ = -2.0;
  max_y_ = 2.0;
  min_z_ = 0.0;
  max_z_ = 6.0;*/

  /*// Basketball
  min_x_ = -4.0;
  max_x_ = 4.0;
  min_y_ = -4.0;
  max_y_ = 4.0;
  min_z_ = 0.0;
  max_z_ = 5.0;*/

  /*// School
  min_x_ = -17;
  max_x_ = 17;
  min_y_ = -12;
  max_y_ = 7.0;
  min_z_ = 0.0;
  max_z_ = 14.5;*/

  /*// Maze
  min_x_ = -10.0;
  max_x_ = 10.0;
  min_y_ = -9.0;
  max_y_ = 9.0;
  min_z_ = 0.0;
  max_z_ = 2.5;*/

  /*// Police Station
  min_x_ = -7;
  max_x_ = 7;
  min_y_ = -8.5;
  max_y_ = 8.5;
  min_z_ = 0.0;
  max_z_ = 9.0;*/

  /*fov_y_rad_ = 1.51844;
  fov_p_rad_ = 1.01229;
  r_max_ = 5.0;
  yaw_samples = 15;*/

  nh_private.param("gain_evaluation/min_x", min_x_, -17.0f);
  nh_private.param("gain_evaluation/max_x", max_x_, 17.0f);
  nh_private.param("gain_evaluation/min_y", min_y_, -12.0f);
  nh_private.param("gain_evaluation/max_y", max_y_, 7.0f);
  nh_private.param("gain_evaluation/min_z", min_z_, 0.0f);
  nh_private.param("gain_evaluation/max_z", max_z_, 14.5f);

  nh_private.param("camera_intrinsics/hfov", fov_y_rad_, 1.51844);
  nh_private.param("camera_intrinsics/vfov", fov_p_rad_, 1.01229);
  nh_private.param("camera_intrinsics/max_distance", r_max_, 5.0);
  nh_private.param("camera_intrinsics/yaw_samples", yaw_samples_, 15);

  nh_private.param("map/voxel_size", dr_, 0.2);

  /*ROS_INFO_STREAM("[GainEvaluator] Bounding Box: \n"
                << "  min_x: " << min_x_ << "\n"
                << "  max_x: " << max_x_ << "\n"
                << "  min_y: " << min_y_ << "\n"
                << "  max_y: " << max_y_ << "\n"
                << "  min_z: " << min_z_ << "\n"
                << "  max_z: " << max_z_);*/
}

double GainEvaluator::getVerticalFoV(double horizontal_fov, int resolution_x, int resolution_y){
  double aspect_ratio = resolution_x/resolution_y;
  double vertical_fov = 2 * std::atan(std::tan(horizontal_fov / 2) / aspect_ratio);
  return vertical_fov;
}

void GainEvaluator::setCameraModelParametersFoV(double horizontal_fov, double vertical_fov,
                                                double min_distance, double max_distance) {
  cam_model_.setIntrinsicsFromFoV(horizontal_fov, vertical_fov, min_distance, max_distance);
}

void GainEvaluator::setCameraModelParametersFocalLength(
    const Eigen::Vector2d& resolution, double focal_length, double min_distance,
    double max_distance) {
  cam_model_.setIntrinsicsFromFocalLength(
      resolution.cast<float>(), focal_length, min_distance, max_distance);
}

void GainEvaluator::setCameraExtrinsics(const voxblox::Transformation& T_C_B) {
  cam_model_.setExtrinsics(T_C_B);
}

void GainEvaluator::setTsdfLayer(voxblox::Layer<voxblox::TsdfVoxel>* tsdf_layer) {
  tsdf_layer_ = tsdf_layer;
  voxel_size_ = tsdf_layer_->voxel_size();
  voxel_size_inv_ = 1.0 / voxel_size_;
  voxels_per_side_ = tsdf_layer_->voxels_per_side();
  voxels_per_side_inv_ = 1.0 / voxels_per_side_;
}

void GainEvaluator::setEsdfMap(voxblox::EsdfMap::Ptr esdf_map) {
  esdf_map_ = esdf_map;
}

void GainEvaluator::getVoxelCenter(Eigen::Vector3d* center, const Eigen::Vector3d& point) {
  voxblox::BlockIndex block_id = esdf_map_->getEsdfLayerPtr()->computeBlockIndexFromCoordinates(point.cast<voxblox::FloatingPoint>());
  *center = voxblox::getOriginPointFromGridIndex(block_id, voxel_size_).cast<double>();
  voxblox::VoxelIndex voxel_id = voxblox::getGridIndexFromPoint<voxblox::VoxelIndex>((point - *center).cast<voxblox::FloatingPoint>(),
          1.0 / voxel_size_);
  *center += voxblox::getCenterPointFromGridIndex(voxel_id, voxel_size_).cast<double>();
}

VoxelStatus GainEvaluator::getVoxelStatus(const Eigen::Vector3d& position) const {
  voxblox::TsdfVoxel* voxel = tsdf_layer_->getVoxelPtrByCoordinates(position.cast<voxblox::FloatingPoint>());
  if (voxel == nullptr) {
    return VoxelStatus::kUnknown;
  }
  if (voxel->weight < 1e-6) {
    return VoxelStatus::kUnknown;
  }
  if (voxel->distance > 0.0) {
    return VoxelStatus::kFree;
  }
  return VoxelStatus::kOccupied;
}

VoxelStatus GainEvaluator::getVisibility(const Eigen::Vector3d& view_point, const Eigen::Vector3d& voxel_to_test,
    bool stop_at_unknown_voxel) const {
  // This involves doing a raycast from view point to voxel to test.
  // Let's get the global voxel coordinates of both.
  const voxblox::Point start_scaled =
      view_point.cast<voxblox::FloatingPoint>() * voxel_size_inv_;
  const voxblox::Point end_scaled =
      voxel_to_test.cast<voxblox::FloatingPoint>() * voxel_size_inv_;

  voxblox::LongIndexVector global_voxel_indices;
  voxblox::castRay(start_scaled, end_scaled, &global_voxel_indices);

  // Iterate over the ray.
  for (const voxblox::GlobalIndex& global_index : global_voxel_indices) {
    voxblox::TsdfVoxel* voxel =
        tsdf_layer_->getVoxelPtrByGlobalIndex(global_index);
    if (voxel == nullptr || voxel->weight < 1e-6) {
      if (stop_at_unknown_voxel) {
        return VoxelStatus::kUnknown;
      }
    } else if (voxel->distance <= 0.0) {
      return VoxelStatus::kOccupied;
    }
  }
  return VoxelStatus::kFree;
}

void GainEvaluator::visualize_frustum(const eth_mav_msgs::EigenTrajectoryPoint& pose, std::vector<geometry_msgs::Point>& points) {
  cam_model_.setBodyPose(voxblox::Transformation(
      pose.orientation_W_B.cast<float>(), pose.position_W.cast<float>()));

  voxblox::Pointcloud lines;
  cam_model_.getBoundingLines(&lines);
  geometry_msgs::Point p1;

  for (size_t i = 0; i < lines.size(); ++i) {
    p1.x = lines[i].x();
    p1.y = lines[i].y();
    p1.z = lines[i].z();
    points.push_back(p1);
  }
}

double GainEvaluator::computeFixedGainRaycasting(const eth_mav_msgs::EigenTrajectoryPoint& pose, int modulus) {
  CHECK_NOTNULL(tsdf_layer_);

  //auto start = std::chrono::high_resolution_clock::now();

  cam_model_.setBodyPose(voxblox::Transformation(
      pose.orientation_W_B.cast<float>(), pose.position_W.cast<float>()));

  // Get the center of the camera to raycast to.
  voxblox::Transformation camera_pose = cam_model_.getCameraPose();
  voxblox::Point camera_center = camera_pose.getPosition();

  double yaw_rad = pose.getYaw();
  double yaw = yaw_rad * 180 / M_PI;

  double gain = 0.0;

  // This function computes the gain
  double fov_y = fov_y_rad_ / M_PI * 180.0f;
  double fov_p = fov_p_rad_ / M_PI * 180.0f;

  double dphi_rad = dr_ / r_max_;
  double dtheta_rad = dr_ / r_max_;
  double dphi = 180.0f * dphi_rad / M_PI, dtheta = 180.0f * dtheta_rad / M_PI;
  double r;
  double phi, theta;
  double phi_rad, theta_rad;

  //voxblox::Point vec;
  Eigen::Vector3d vec;
  double min_x = static_cast<double>(min_x_);
  double min_y = static_cast<double>(min_y_);
  double min_z = static_cast<double>(min_z_);
  double max_x = static_cast<double>(max_x_);
  double max_y = static_cast<double>(max_y_);
  double max_z = static_cast<double>(max_z_);

  int id = 0;
  for (theta = yaw - fov_y/2; theta < yaw + fov_y/2; theta += dtheta) {
    theta_rad = M_PI * theta / 180.0f;
    for (phi = 90 - fov_p / 2; phi < 90 + fov_p / 2; phi += dphi) {
      phi_rad = M_PI * phi / 180.0f;

      double g = 0;
      bool occupied_ray = false;
      for (r = 0; r < r_max_; r += dr_) {
        vec[0] = pose.position_W[0] + r * cos(theta_rad) * sin(phi_rad);
        vec[1] = pose.position_W[1] + r * sin(theta_rad) * sin(phi_rad);
        vec[2] = pose.position_W[2] + r * cos(phi_rad);

        if (vec[0] < min_x || vec[0] > max_x || 
        vec[1] < min_y || vec[1] > max_y || 
        vec[2] < min_z || vec[2] > max_z) {
          continue;
        }

        VoxelStatus node = getVoxelStatus(vec);

        if (node == kOccupied) {
          occupied_ray = true;
          break;
        } else if (node == kFree) {
          continue;
        } else if (node == kUnknown) {
          g += (2 * r * r * dr_ + 1 / 6 * dr_ * dr_ * dr_) * dtheta_rad * sin(phi_rad) * sin(dphi_rad / 2);
        }
      }
      gain += g;
    }
  }

  //auto end = std::chrono::high_resolution_clock::now();
  //std::chrono::duration<double> elapsed = end - start;
  //ROS_INFO("[AEPlanner]: RayCasting took: %f seconds.", elapsed.count());
  //std::cout << "AEPGain took " << elapsed.count() << " seconds." << std::endl;

  return gain;
}

std::pair<double, double> GainEvaluator::computeGainRaycasting(const eth_mav_msgs::EigenTrajectoryPoint& pose, int modulus) {
  CHECK_NOTNULL(tsdf_layer_);

  //auto start = std::chrono::high_resolution_clock::now();

  cam_model_.setBodyPose(voxblox::Transformation(
      pose.orientation_W_B.cast<float>(), pose.position_W.cast<float>()));

  // Get the center of the camera to raycast to.
  voxblox::Transformation camera_pose = cam_model_.getCameraPose();
  voxblox::Point camera_center = camera_pose.getPosition();

  double gain = 0.0;

  // This function computes the gain
  double fov_y = fov_y_rad_ / M_PI * 180.0f;
  double fov_p = fov_p_rad_ / M_PI * 180.0f;

  double dphi_rad = dr_ / r_max_;
  double dtheta_rad = dr_ / r_max_;
  double dphi = 180.0f * dphi_rad / M_PI, dtheta = 180.0f * dtheta_rad / M_PI;
  double r;
  double phi, theta;
  double phi_rad, theta_rad;

  std::map<int, double> gain_per_yaw;

  //voxblox::Point vec;
  Eigen::Vector3d vec;
  double min_x = static_cast<double>(min_x_);
  double min_y = static_cast<double>(min_y_);
  double min_z = static_cast<double>(min_z_);
  double max_x = static_cast<double>(max_x_);
  double max_y = static_cast<double>(max_y_);
  double max_z = static_cast<double>(max_z_);

  int id = 0;
  for (theta = -180; theta < 180; theta += dtheta) {
    theta_rad = M_PI * theta / 180.0f;
    for (phi = 90 - fov_p / 2; phi < 90 + fov_p / 2; phi += dphi) {
      phi_rad = M_PI * phi / 180.0f;

      double g = 0;
      bool occupied_ray = false;
      for (r = 0; r < r_max_; r += dr_) {
        vec[0] = pose.position_W[0] + r * cos(theta_rad) * sin(phi_rad);
        vec[1] = pose.position_W[1] + r * sin(theta_rad) * sin(phi_rad);
        vec[2] = pose.position_W[2] + r * cos(phi_rad);

        if (vec[0] < min_x || vec[0] > max_x || 
        vec[1] < min_y || vec[1] > max_y || 
        vec[2] < min_z || vec[2] > max_z) {
          continue;
        }

        VoxelStatus node = getVoxelStatus(vec);

        if (node == kOccupied) {
          occupied_ray = true;
          break;
        } else if (node == kFree) {
          continue;
        } else if (node == kUnknown) {
          g += (2 * r * r * dr_ + 1 / 6 * dr_ * dr_ * dr_) * dtheta_rad * sin(phi_rad) * sin(dphi_rad / 2);
        }
      }
      gain += g;
      gain_per_yaw[theta] += g;
    }
  }

  //auto intermediate = std::chrono::high_resolution_clock::now();

  int best_yaw = 0;
  double best_yaw_score = 0;
  for (int yaw = -180; yaw < 180; yaw++)
  {
    double yaw_score = 0;
    for (int fov = -fov_y / 2; fov < fov_y / 2; fov++)
    {
      int theta = yaw + fov;
      if (theta < -180)
        theta += 360;
      if (theta > 180)
        theta -= 360;
      yaw_score += gain_per_yaw[theta];
    }

    if (best_yaw_score < yaw_score)
    {
      best_yaw_score = yaw_score;
      best_yaw = yaw;
    }
  }

  double h_max = fov_y / M_PI * 180;
  double v_max = fov_p / M_PI * 180;

  gain = best_yaw_score;

  double yaw = M_PI * best_yaw / 180.f;

  //auto end = std::chrono::high_resolution_clock::now();
  //std::chrono::duration<double> elapsed = end - start;
  //std::chrono::duration<double> intermediate_elapsed = intermediate - start;
  //std::cout << "First Part AEPGain took " << intermediate_elapsed.count() << " seconds." << std::endl;
  //std::cout << "AEPGain took " << elapsed.count() << " seconds." << std::endl;

  return std::make_pair(gain, yaw);
}

std::pair<double, double> GainEvaluator::computeGainOptimizedRaycasting(const eth_mav_msgs::EigenTrajectoryPoint& pose, int modulus) {
  CHECK_NOTNULL(tsdf_layer_);

  //auto start = std::chrono::high_resolution_clock::now();

  cam_model_.setBodyPose(voxblox::Transformation(
      pose.orientation_W_B.cast<float>(), pose.position_W.cast<float>()));

  // Get the center of the camera to raycast to.
  voxblox::Transformation camera_pose = cam_model_.getCameraPose();
  voxblox::Point camera_center = camera_pose.getPosition();

  double gain = 0.0;

  // This function computes the gain
  double fov_y = fov_y_rad_ / M_PI * 180.0f;
  double fov_p = fov_p_rad_ / M_PI * 180.0f;

  double dphi_rad = dr_ / r_max_;
  double dtheta_rad = dr_ / r_max_;
  double dphi = 180.0f * dphi_rad / M_PI, dtheta = 180.0f * dtheta_rad / M_PI;
  double r;
  double phi, theta;
  double phi_rad, theta_rad;

  std::map<int, double> gain_per_yaw;

  //voxblox::Point vec;
  Eigen::Vector3d vec;
  double min_x = static_cast<double>(min_x_);
  double min_y = static_cast<double>(min_y_);
  double min_z = static_cast<double>(min_z_);
  double max_x = static_cast<double>(max_x_);
  double max_y = static_cast<double>(max_y_);
  double max_z = static_cast<double>(max_z_);

  int id = 0;
  for (theta = -180; theta < 180; theta += dtheta) {
    theta_rad = M_PI * theta / 180.0f;
    for (phi = 90 - fov_p / 2; phi < 90 + fov_p / 2; phi += dphi) {
      phi_rad = M_PI * phi / 180.0f;

      double g = 0;
      bool occupied_ray = false;
      for (r = 0; r < r_max_; r += dr_) {
        vec[0] = pose.position_W[0] + r * cos(theta_rad) * sin(phi_rad);
        vec[1] = pose.position_W[1] + r * sin(theta_rad) * sin(phi_rad);
        vec[2] = pose.position_W[2] + r * cos(phi_rad);

        if (vec[0] < min_x || vec[0] > max_x || 
        vec[1] < min_y || vec[1] > max_y || 
        vec[2] < min_z || vec[2] > max_z) {
          continue;
        }

        VoxelStatus node = getVoxelStatus(vec);

        if (node == kOccupied) {
          occupied_ray = true;
          break;
        } else if (node == kFree) {
          continue;
        } else if (node == kUnknown) {
          g += (2 * r * r * dr_ + 1 / 6 * dr_ * dr_ * dr_) * dtheta_rad * sin(phi_rad) * sin(dphi_rad / 2);
        }
      }

      gain += g;
      gain_per_yaw[theta] += g;
    }
  }

  //auto intermediate = std::chrono::high_resolution_clock::now();

  double best_gain = 0;
  double best_yaw = 0;

  int min_yaw_samples = ceil(2 * M_PI / fov_y_rad_);

  std::vector<double> yaws;
  std::vector<double> gains;
  double min_yaw_step = 2 * M_PI / min_yaw_samples;
  double yaw_step = 1; // degree
  int aditional_angles = (360 - min_yaw_samples) / min_yaw_samples;

  //auto start = std::chrono::high_resolution_clock::now();

  for (int k = 0; k < min_yaw_samples; ++k) {
    double yaw_optimized = k * min_yaw_step / M_PI * 180.0f;
    double gain_optimized = 0;
    for (int fov = -fov_y / 2; fov < fov_y / 2; fov++) {
      int theta = yaw_optimized + fov;
      if (theta < -180)
        theta += 360;
      if (theta > 180)
        theta -= 360;
      gain_optimized += gain_per_yaw[theta];
    }

    yaws.push_back(yaw_optimized);
    gains.push_back(gain_optimized);

    if (gain_optimized > best_gain) {
      best_gain = gain_optimized;
      best_yaw = yaw_optimized;
    }
  }

  // Create a vector to store the filtered yaws
  std::vector<double> filteredYaws;

  for (int i = 0; i < min_yaw_samples; ++i) {
    //int prev = (i - 1 + min_yaw_samples) % min_yaw_samples;
    int next = (i + 1) % min_yaw_samples;

    if ((gains[i] + gains[next] > best_gain)) {
      filteredYaws.push_back(yaws[i]);
    }
  }

  for (int j = 0; j < filteredYaws.size(); ++j) {
    for (int l = 0; l < aditional_angles; ++l) {
      double yaw_optimized = filteredYaws[j] + yaw_step * (l + 1);
      double gain_optimized = 0;
      for (int fov = -fov_y / 2; fov < fov_y / 2; fov++) {
        int theta = yaw_optimized + fov;
        if (theta < -180)
          theta += 360;
        if (theta > 180)
          theta -= 360;
        gain_optimized += gain_per_yaw[theta];
      }

      if (gain_optimized > best_gain) {
        best_gain = gain_optimized;
        best_yaw = yaw_optimized;
      }
    }
  }

  gain = best_gain;
  double yaw = M_PI * best_yaw / 180.f;

  //auto end = std::chrono::high_resolution_clock::now();
  //std::chrono::duration<double> elapsed = end - start;
  //std::chrono::duration<double> intermediate_elapsed = intermediate - start;
  //std::cout << "First Part AEPGain took " << intermediate_elapsed.count() << " seconds." << std::endl;
  //std::cout << "AEPGain took " << elapsed.count() << " seconds." << std::endl;

  return std::make_pair(gain, yaw);
}

std::pair<double, double> GainEvaluator::computeGainRaycastingFromSampledYaw(eth_mav_msgs::EigenTrajectoryPoint& position) {
  double best_gain = 0;
  double best_yaw = 0;

  //auto start = std::chrono::high_resolution_clock::now();

  for (int k = 0; k < yaw_samples_; ++k) {
    double yaw = k * 2 * M_PI / yaw_samples_;
    //position.position_W = node->point.head(3);
    position.setFromYaw(yaw);
    double gain = computeFixedGainRaycasting(position);
    if (gain > best_gain) {
      best_gain = gain;
      best_yaw = yaw;
    }
  }

  //auto end = std::chrono::high_resolution_clock::now();
  //std::chrono::duration<double> elapsed = end - start;

  return std::make_pair(best_gain, best_yaw);
}

std::pair<double, double> GainEvaluator::computeGainRaycastingFromOptimizedSampledYaw(eth_mav_msgs::EigenTrajectoryPoint& position) {
  double best_gain = 0;
  double best_yaw = 0;

  int min_yaw_samples = ceil(2 * M_PI / fov_y_rad_);

  std::vector<double> yaws;
  std::vector<double> gains;
  double min_yaw_step = 2 * M_PI / min_yaw_samples;
  double yaw_step = 2 * M_PI / yaw_samples_;
  int aditional_angles = (yaw_samples_ - min_yaw_samples) / min_yaw_samples;

  //auto start = std::chrono::high_resolution_clock::now();

  for (int k = 0; k < min_yaw_samples; ++k) {
    double yaw = k * min_yaw_step;
    position.setFromYaw(yaw);
    double gain = computeFixedGainRaycasting(position);

    yaws.push_back(yaw);
    gains.push_back(gain);

    if (gain > best_gain) {
      best_gain = gain;
      best_yaw = yaw;
    }
  }

  // Create a vector to store the filtered yaws
  std::vector<double> filteredYaws;

  for (int i = 0; i < min_yaw_samples; ++i) {
    //int prev = (i - 1 + min_yaw_samples) % min_yaw_samples;
    int next = (i + 1) % min_yaw_samples;

    if ((gains[i] + gains[next] > best_gain)) {
      filteredYaws.push_back(yaws[i]);
    }
  }

  for (int j = 0; j < filteredYaws.size(); ++j) {
    for (int l = 0; l < aditional_angles; ++l) {
      double yaw = filteredYaws[j] + yaw_step * (l + 1);
      position.setFromYaw(yaw);
      double gain = computeFixedGainRaycasting(position);

      if (gain > best_gain) {
        best_gain = gain;
        best_yaw = yaw;
      }
    }
  }

  //auto end = std::chrono::high_resolution_clock::now();
  //std::chrono::duration<double> elapsed = end - start;

  return std::make_pair(best_gain, best_yaw);
}

double GainEvaluator::computeGainFixedAngleAEP(const eth_mav_msgs::EigenTrajectoryPoint& pose, int modulus) {
  CHECK_NOTNULL(tsdf_layer_);

  //auto start = std::chrono::high_resolution_clock::now();

  cam_model_.setBodyPose(voxblox::Transformation(
      pose.orientation_W_B.cast<float>(), pose.position_W.cast<float>()));

  // Get the center of the camera to raycast to.
  voxblox::Transformation camera_pose = cam_model_.getCameraPose();
  voxblox::Point camera_center = camera_pose.getPosition();

  double yaw_rad = pose.getYaw();
  double yaw = yaw_rad * 180 / M_PI;

  double gain = 0.0;

  // This function computes the gain
  double fov_y = fov_y_rad_ / M_PI * 180.0f;
  double fov_p = fov_p_rad_ / M_PI * 180.0f;

  double dphi = 10, dtheta = 10;
  double dphi_rad = M_PI * dphi / 180.0f, dtheta_rad = M_PI * dtheta / 180.0f;
  double r;
  int phi, theta;
  double phi_rad, theta_rad;

  //voxblox::Point vec;
  Eigen::Vector3d vec;
  double min_x = static_cast<double>(min_x_);
  double min_y = static_cast<double>(min_y_);
  double min_z = static_cast<double>(min_z_);
  double max_x = static_cast<double>(max_x_);
  double max_y = static_cast<double>(max_y_);
  double max_z = static_cast<double>(max_z_);

  int id = 0;
  for (theta = yaw - fov_y/2; theta < yaw + fov_y/2; theta += dtheta) {
    theta_rad = M_PI * theta / 180.0f;
    for (phi = 90 - fov_p / 2; phi < 90 + fov_p / 2; phi += dphi) {
      phi_rad = M_PI * phi / 180.0f;

      double g = 0.0;
      bool occupied_ray = false;
      for (r = 0; r < r_max_; r += dr_) {
        vec[0] = pose.position_W[0] + r * cos(theta_rad) * sin(phi_rad);
        vec[1] = pose.position_W[1] + r * sin(theta_rad) * sin(phi_rad);
        vec[2] = pose.position_W[2] + r * cos(phi_rad);

        if (vec[0] < min_x || vec[0] > max_x || 
        vec[1] < min_y || vec[1] > max_y || 
        vec[2] < min_z || vec[2] > max_z) {
          continue;
        }

        VoxelStatus node = getVoxelStatus(vec);

        if (node == kOccupied) {
          occupied_ray = true;
          break;
        } else if (node == kFree) {
          continue;
        } else if (node == kUnknown) {
          g += (2 * r * r * dr_ + 1 / 6 * dr_ * dr_ * dr_) * dtheta_rad * sin(phi_rad) * sin(dphi_rad / 2);
        }
      }

      gain += g;
    }
  }

  //auto end = std::chrono::high_resolution_clock::now();
  //std::chrono::duration<double> elapsed = end - start;
  //std::cout << "AEPGain took " << elapsed.count() << " seconds." << std::endl;

  return gain;
}

double GainEvaluator::computeGainFixedAngleAEP(const eth_mav_msgs::EigenTrajectoryPoint& pose, Eigen::Vector3d offset, int modulus) {
  CHECK_NOTNULL(tsdf_layer_);

  //auto start = std::chrono::high_resolution_clock::now();

  cam_model_.setBodyPose(voxblox::Transformation(
      pose.orientation_W_B.cast<float>(), pose.position_W.cast<float>()));

  // Get the center of the camera to raycast to.
  voxblox::Transformation camera_pose = cam_model_.getCameraPose();
  voxblox::Point camera_center = camera_pose.getPosition();

  double yaw_rad = pose.getYaw();
  double yaw = yaw_rad * 180 / M_PI;

  double gain = 0.0;

  // This function computes the gain
  double fov_y = fov_y_rad_ / M_PI * 180.0f;
  double fov_p = fov_p_rad_ / M_PI * 180.0f;

  double dphi = 10, dtheta = 10;
  double dphi_rad = M_PI * dphi / 180.0f, dtheta_rad = M_PI * dtheta / 180.0f;
  double r;
  int phi, theta;
  double phi_rad, theta_rad;

  //voxblox::Point vec;
  Eigen::Vector3d vec;
  double min_x = static_cast<double>(min_x_);
  double min_y = static_cast<double>(min_y_);
  double min_z = static_cast<double>(min_z_);
  double max_x = static_cast<double>(max_x_);
  double max_y = static_cast<double>(max_y_);
  double max_z = static_cast<double>(max_z_);

  int id = 0;
  for (theta = yaw - fov_y/2; theta < yaw + fov_y/2; theta += dtheta) {
    theta_rad = M_PI * theta / 180.0f;
    for (phi = 90 - fov_p / 2; phi < 90 + fov_p / 2; phi += dphi) {
      phi_rad = M_PI * phi / 180.0f;

      double g = 0.0;
      bool occupied_ray = false;
      for (r = 0; r < r_max_; r += dr_) {
        vec[0] = pose.position_W[0] + r * cos(theta_rad) * sin(phi_rad);
        vec[1] = pose.position_W[1] + r * sin(theta_rad) * sin(phi_rad);
        vec[2] = pose.position_W[2] + r * cos(phi_rad);

        if (vec[0] < min_x + offset[0] || vec[0] > max_x + offset[0] || 
        vec[1] < min_y + offset[1] || vec[1] > max_y + offset[1] || 
        vec[2] < min_z + offset[2] || vec[2] > max_z + offset[2]) {
          continue;
        }

        VoxelStatus node = getVoxelStatus(vec);

        if (node == kOccupied) {
          occupied_ray = true;
          break;
        } else if (node == kFree) {
          continue;
        } else if (node == kUnknown) {
          g += (2 * r * r * dr_ + 1 / 6 * dr_ * dr_ * dr_) * dtheta_rad * sin(phi_rad) * sin(dphi_rad / 2);
        }
      }
      gain += g;
    }
  }

  //auto end = std::chrono::high_resolution_clock::now();
  //std::chrono::duration<double> elapsed = end - start;
  //std::cout << "AEPGain took " << elapsed.count() << " seconds." << std::endl;

  return gain;
}

std::pair<double, double> GainEvaluator::computeGainAEP(const eth_mav_msgs::EigenTrajectoryPoint& pose, int modulus) {
  CHECK_NOTNULL(tsdf_layer_);

  //auto start = std::chrono::high_resolution_clock::now();

  cam_model_.setBodyPose(voxblox::Transformation(
      pose.orientation_W_B.cast<float>(), pose.position_W.cast<float>()));

  // Get the center of the camera to raycast to.
  voxblox::Transformation camera_pose = cam_model_.getCameraPose();
  voxblox::Point camera_center = camera_pose.getPosition();

  double gain = 0.0;

  // This function computes the gain
  double fov_y = fov_y_rad_ / M_PI * 180.0f;
  double fov_p = fov_p_rad_ / M_PI * 180.0f;

  double dphi = 10, dtheta = 10;
  double dphi_rad = M_PI * dphi / 180.0f, dtheta_rad = M_PI * dtheta / 180.0f;
  double r;
  int phi, theta;
  double phi_rad, theta_rad;

  std::map<int, double> gain_per_yaw;

  //voxblox::Point vec;
  Eigen::Vector3d vec;
  double min_x = static_cast<double>(min_x_);
  double min_y = static_cast<double>(min_y_);
  double min_z = static_cast<double>(min_z_);
  double max_x = static_cast<double>(max_x_);
  double max_y = static_cast<double>(max_y_);
  double max_z = static_cast<double>(max_z_);

  int id = 0;
  for (theta = -180; theta < 180; theta += dtheta) {
    theta_rad = M_PI * theta / 180.0f;
    for (phi = 90 - fov_p / 2; phi < 90 + fov_p / 2; phi += dphi) {
      phi_rad = M_PI * phi / 180.0f;

      double g = 0;
      bool occupied_ray = false;
      for (r = 0; r < r_max_; r += dr_) {
        vec[0] = pose.position_W[0] + r * cos(theta_rad) * sin(phi_rad);
        vec[1] = pose.position_W[1] + r * sin(theta_rad) * sin(phi_rad);
        vec[2] = pose.position_W[2] + r * cos(phi_rad);

        if (vec[0] < min_x || vec[0] > max_x || 
        vec[1] < min_y || vec[1] > max_y || 
        vec[2] < min_z || vec[2] > max_z) {
          continue;
        }

        VoxelStatus node = getVoxelStatus(vec);

        if (node == kOccupied) {
          occupied_ray = true;
          break;
        } else if (node == kFree) {
          continue;
        } else if (node == kUnknown) {
          g += (2 * r * r * dr_ + 1 / 6 * dr_ * dr_ * dr_) * dtheta_rad * sin(phi_rad) * sin(dphi_rad / 2);
        }
      }
      gain += g;
      gain_per_yaw[theta] += g;
    }
  }

  //auto intermediate = std::chrono::high_resolution_clock::now();

  int best_yaw = 0;
  double best_yaw_score = 0;
  for (int yaw = -180; yaw < 180; yaw++)
  {
    double yaw_score = 0;
    for (int fov = -fov_y / 2; fov < fov_y / 2; fov++)
    {
      int theta = yaw + fov;
      if (theta < -180)
        theta += 360;
      if (theta > 180)
        theta -= 360;
      yaw_score += gain_per_yaw[theta];
    }

    if (best_yaw_score < yaw_score)
    {
      best_yaw_score = yaw_score;
      best_yaw = yaw;
    }
  }

  double h_max = fov_y / M_PI * 180;
  double v_max = fov_p / M_PI * 180;

  gain = best_yaw_score;

  double yaw = M_PI * best_yaw / 180.f;

  //auto end = std::chrono::high_resolution_clock::now();
  //std::chrono::duration<double> elapsed = end - start;
  //std::chrono::duration<double> intermediate_elapsed = intermediate - start;
  //std::cout << "First Part AEPGain took " << intermediate_elapsed.count() << " seconds." << std::endl;
  //std::cout << "AEPGain took " << elapsed.count() << " seconds." << std::endl;

  return std::make_pair(gain, yaw);
}

std::pair<double, double> GainEvaluator::computeGainOptimizedAEP(const eth_mav_msgs::EigenTrajectoryPoint& pose, int modulus) {
  CHECK_NOTNULL(tsdf_layer_);

  //auto start = std::chrono::high_resolution_clock::now();

  cam_model_.setBodyPose(voxblox::Transformation(
      pose.orientation_W_B.cast<float>(), pose.position_W.cast<float>()));

  // Get the center of the camera to raycast to.
  voxblox::Transformation camera_pose = cam_model_.getCameraPose();
  voxblox::Point camera_center = camera_pose.getPosition();

  double gain = 0.0;

  // This function computes the gain
  double fov_y = fov_y_rad_ / M_PI * 180.0f;
  double fov_p = fov_p_rad_ / M_PI * 180.0f;

  double dphi = 10, dtheta = 10;
  double dphi_rad = M_PI * dphi / 180.0f, dtheta_rad = M_PI * dtheta / 180.0f;
  double r;
  int phi, theta;
  double phi_rad, theta_rad;

  std::map<int, double> gain_per_yaw;

  //voxblox::Point vec;
  Eigen::Vector3d vec;
  double min_x = static_cast<double>(min_x_);
  double min_y = static_cast<double>(min_y_);
  double min_z = static_cast<double>(min_z_);
  double max_x = static_cast<double>(max_x_);
  double max_y = static_cast<double>(max_y_);
  double max_z = static_cast<double>(max_z_);

  int id = 0;
  for (theta = -180; theta < 180; theta += dtheta) {
    theta_rad = M_PI * theta / 180.0f;
    for (phi = 90 - fov_p / 2; phi < 90 + fov_p / 2; phi += dphi) {
      phi_rad = M_PI * phi / 180.0f;

      double g = 0;
      bool occupied_ray = false;
      for (r = 0; r < r_max_; r += dr_) {
        vec[0] = pose.position_W[0] + r * cos(theta_rad) * sin(phi_rad);
        vec[1] = pose.position_W[1] + r * sin(theta_rad) * sin(phi_rad);
        vec[2] = pose.position_W[2] + r * cos(phi_rad);

        if (vec[0] < min_x || vec[0] > max_x || 
        vec[1] < min_y || vec[1] > max_y || 
        vec[2] < min_z || vec[2] > max_z) {
          continue;
        }

        VoxelStatus node = getVoxelStatus(vec);

        if (node == kOccupied) {
          occupied_ray = true;
          break;
        } else if (node == kFree) {
          continue;
        } else if (node == kUnknown) {
          g += (2 * r * r * dr_ + 1 / 6 * dr_ * dr_ * dr_) * dtheta_rad * sin(phi_rad) * sin(dphi_rad / 2);
        }
      }
      gain += g;
      gain_per_yaw[theta] += g;
    }
  }

  //auto intermediate = std::chrono::high_resolution_clock::now();

  double best_gain = 0;
  double best_yaw = 0;

  int min_yaw_samples = ceil(2 * M_PI / fov_y_rad_);

  std::vector<double> yaws;
  std::vector<double> gains;
  double min_yaw_step = 2 * M_PI / min_yaw_samples;
  double yaw_step = 1; // degree
  int aditional_angles = (360 - min_yaw_samples) / min_yaw_samples;

  for (int k = 0; k < min_yaw_samples; ++k) {
    double yaw_optimized = k * min_yaw_step / M_PI * 180.0f;
    double gain_optimized = 0;
    for (int fov = -fov_y / 2; fov < fov_y / 2; fov++) {
      int theta = yaw_optimized + fov;
      if (theta < -180)
        theta += 360;
      if (theta > 180)
        theta -= 360;
      gain_optimized += gain_per_yaw[theta];
    }

    yaws.push_back(yaw_optimized);
    gains.push_back(gain_optimized);

    if (gain_optimized > best_gain) {
      best_gain = gain_optimized;
      best_yaw = yaw_optimized;
    }
  }

  // Create a vector to store the filtered yaws
  std::vector<double> filteredYaws;

  for (int i = 0; i < min_yaw_samples; ++i) {
    int next = (i + 1) % min_yaw_samples;

    if ((gains[i] + gains[next] > best_gain)) {
      filteredYaws.push_back(yaws[i]);
    }
  }

  for (int j = 0; j < filteredYaws.size(); ++j) {
    for (int l = 0; l < aditional_angles; ++l) {
      double yaw_optimized = filteredYaws[j] + yaw_step * (l + 1);
      double gain_optimized = 0;
      for (int fov = -fov_y / 2; fov < fov_y / 2; fov++) {
        int theta = yaw_optimized + fov;
        if (theta < -180)
          theta += 360;
        if (theta > 180)
          theta -= 360;
        gain_optimized += gain_per_yaw[theta];
      }

      if (gain_optimized > best_gain) {
        best_gain = gain_optimized;
        best_yaw = yaw_optimized;
      }
    }
  }

  gain = best_gain;
  double yaw = M_PI * best_yaw / 180.f;

  //auto end = std::chrono::high_resolution_clock::now();
  //std::chrono::duration<double> elapsed = end - start;
  //std::chrono::duration<double> intermediate_elapsed = intermediate - start;
  //std::cout << "First Part AEPGain took " << intermediate_elapsed.count() << " seconds." << std::endl;
  //std::cout << "AEPGain took " << elapsed.count() << " seconds." << std::endl;

  return std::make_pair(gain, yaw);
}

std::pair<double, double> GainEvaluator::computeGainOptimizedAEP(const eth_mav_msgs::EigenTrajectoryPoint& pose, Eigen::Vector3d offset, int modulus) {
  CHECK_NOTNULL(tsdf_layer_);

  //auto start = std::chrono::high_resolution_clock::now();

  cam_model_.setBodyPose(voxblox::Transformation(
      pose.orientation_W_B.cast<float>(), pose.position_W.cast<float>()));

  // Get the center of the camera to raycast to.
  voxblox::Transformation camera_pose = cam_model_.getCameraPose();
  voxblox::Point camera_center = camera_pose.getPosition();

  double gain = 0.0;

  // This function computes the gain
  double fov_y = fov_y_rad_ / M_PI * 180.0f;
  double fov_p = fov_p_rad_ / M_PI * 180.0f;

  double dphi = 10, dtheta = 10;
  double dphi_rad = M_PI * dphi / 180.0f, dtheta_rad = M_PI * dtheta / 180.0f;
  double r;
  int phi, theta;
  double phi_rad, theta_rad;

  std::map<int, double> gain_per_yaw;

  //voxblox::Point vec;
  Eigen::Vector3d vec;
  double min_x = static_cast<double>(min_x_);
  double min_y = static_cast<double>(min_y_);
  double min_z = static_cast<double>(min_z_);
  double max_x = static_cast<double>(max_x_);
  double max_y = static_cast<double>(max_y_);
  double max_z = static_cast<double>(max_z_);

  int id = 0;
  for (theta = -180; theta < 180; theta += dtheta) {
    theta_rad = M_PI * theta / 180.0f;
    for (phi = 90 - fov_p / 2; phi < 90 + fov_p / 2; phi += dphi) {
      phi_rad = M_PI * phi / 180.0f;

      double g = 0;
      bool occupied_ray = false;
      for (r = 0; r < r_max_; r += dr_) {
        vec[0] = pose.position_W[0] + r * cos(theta_rad) * sin(phi_rad);
        vec[1] = pose.position_W[1] + r * sin(theta_rad) * sin(phi_rad);
        vec[2] = pose.position_W[2] + r * cos(phi_rad);

        if (vec[0] < min_x + offset[0] || vec[0] > max_x + offset[0] || 
        vec[1] < min_y + offset[1] || vec[1] > max_y + offset[1] || 
        vec[2] < min_z + offset[2] || vec[2] > max_z + offset[2]) {
          continue;
        }

        VoxelStatus node = getVoxelStatus(vec);

        if (node == kOccupied) {
          occupied_ray = true;
          break;
        } else if (node == kFree) {
          continue;
        } else if (node == kUnknown) {
          g += (2 * r * r * dr_ + 1 / 6 * dr_ * dr_ * dr_) * dtheta_rad * sin(phi_rad) * sin(dphi_rad / 2);
        }
      }
      gain += g;
      gain_per_yaw[theta] += g;
    }
  }

  //auto intermediate = std::chrono::high_resolution_clock::now();

  double best_gain = 0;
  double best_yaw = 0;

  int min_yaw_samples = ceil(2 * M_PI / fov_y_rad_);

  std::vector<double> yaws;
  std::vector<double> gains;
  double min_yaw_step = 2 * M_PI / min_yaw_samples;
  double yaw_step = 1; // degree
  int aditional_angles = (360 - min_yaw_samples) / min_yaw_samples;

  for (int k = 0; k < min_yaw_samples; ++k) {
    double yaw_optimized = k * min_yaw_step / M_PI * 180.0f;
    double gain_optimized = 0;
    for (int fov = -fov_y / 2; fov < fov_y / 2; fov++) {
      int theta = yaw_optimized + fov;
      if (theta < -180)
        theta += 360;
      if (theta > 180)
        theta -= 360;
      gain_optimized += gain_per_yaw[theta];
    }

    yaws.push_back(yaw_optimized);
    gains.push_back(gain_optimized);

    if (gain_optimized > best_gain) {
      best_gain = gain_optimized;
      best_yaw = yaw_optimized;
    }
  }

  // Create a vector to store the filtered yaws
  std::vector<double> filteredYaws;

  for (int i = 0; i < min_yaw_samples; ++i) {
    //int prev = (i - 1 + min_yaw_samples) % min_yaw_samples;
    int next = (i + 1) % min_yaw_samples;

    if ((gains[i] + gains[next] > best_gain)) {
      filteredYaws.push_back(yaws[i]);
    }
  }

  for (int j = 0; j < filteredYaws.size(); ++j) {
    for (int l = 0; l < aditional_angles; ++l) {
      double yaw_optimized = filteredYaws[j] + yaw_step * (l + 1);
      double gain_optimized = 0;
      for (int fov = -fov_y / 2; fov < fov_y / 2; fov++) {
        int theta = yaw_optimized + fov;
        if (theta < -180)
          theta += 360;
        if (theta > 180)
          theta -= 360;
        gain_optimized += gain_per_yaw[theta];
      }

      if (gain_optimized > best_gain) {
        best_gain = gain_optimized;
        best_yaw = yaw_optimized;
      }
    }
  }

  gain = best_gain;
  double yaw = M_PI * best_yaw / 180.f;

  //auto end = std::chrono::high_resolution_clock::now();
  //std::chrono::duration<double> elapsed = end - start;
  //std::chrono::duration<double> intermediate_elapsed = intermediate - start;
  //std::cout << "First Part AEPGain took " << intermediate_elapsed.count() << " seconds." << std::endl;
  //std::cout << "AEPGain took " << elapsed.count() << " seconds." << std::endl;

  return std::make_pair(gain, yaw);
}

std::pair<double, double> GainEvaluator::computeGainFromSampledYawAEP(eth_mav_msgs::EigenTrajectoryPoint& position) {
  double best_gain = 0;
  double best_yaw = 0;

  //auto start = std::chrono::high_resolution_clock::now();

  for (int k = 0; k < yaw_samples_; ++k) {
    double yaw = k * 2 * M_PI / yaw_samples_;
    //position.position_W = node->point.head(3);
    position.setFromYaw(yaw);
    double gain = computeGainFixedAngleAEP(position);
    if (gain > best_gain) {
      best_gain = gain;
      best_yaw = yaw;
    }
  }

  //auto end = std::chrono::high_resolution_clock::now();
  //std::chrono::duration<double> elapsed = end - start;

  return std::make_pair(best_gain, best_yaw);
}

std::pair<double, double> GainEvaluator::computeGainFromOptimizedSampledYawAEP(eth_mav_msgs::EigenTrajectoryPoint& position) {
  double best_gain = 0;
  double best_yaw = 0;

  int min_yaw_samples = ceil(2 * M_PI / fov_y_rad_);

  std::vector<double> yaws;
  std::vector<double> gains;
  double min_yaw_step = 2 * M_PI / min_yaw_samples;
  double yaw_step = 2 * M_PI / yaw_samples_;
  int aditional_angles = (yaw_samples_ - min_yaw_samples) / min_yaw_samples;

  //auto start = std::chrono::high_resolution_clock::now();

  for (int k = 0; k < min_yaw_samples; ++k) {
    double yaw = k * min_yaw_step;
    position.setFromYaw(yaw);
    double gain = computeGainFixedAngleAEP(position);

    yaws.push_back(yaw);
    gains.push_back(gain);

    if (gain > best_gain) {
      best_gain = gain;
      best_yaw = yaw;
    }
  }

  // Create a vector to store the filtered yaws
  std::vector<double> filteredYaws;

  for (int i = 0; i < min_yaw_samples; ++i) {
    int next = (i + 1) % min_yaw_samples;

    if ((gains[i] + gains[next] > best_gain)) {
      filteredYaws.push_back(yaws[i]);
    }
  }

  for (int j = 0; j < filteredYaws.size(); ++j) {
    for (int l = 0; l < aditional_angles; ++l) {
      double yaw = filteredYaws[j] + yaw_step * (l + 1);
      position.setFromYaw(yaw);
      double gain = computeGainFixedAngleAEP(position);

      if (gain > best_gain) {
        best_gain = gain;
        best_yaw = yaw;
      }
    }
  }

  //auto end = std::chrono::high_resolution_clock::now();
  //std::chrono::duration<double> elapsed = end - start;

  return std::make_pair(best_gain, best_yaw);
}

void GainEvaluator::visualizeGainAEP(const eth_mav_msgs::EigenTrajectoryPoint& pose, voxblox::Pointcloud& voxels) {
  CHECK_NOTNULL(tsdf_layer_);

  //auto start = std::chrono::high_resolution_clock::now();

  cam_model_.setBodyPose(voxblox::Transformation(
      pose.orientation_W_B.cast<float>(), pose.position_W.cast<float>()));

  // Get the center of the camera to raycast to.
  voxblox::Transformation camera_pose = cam_model_.getCameraPose();
  voxblox::Point camera_center = camera_pose.getPosition();
  double yaw_rad = pose.getYaw();
  double yaw = yaw_rad * 180 / M_PI;

  double gain = 0.0;

  // This function computes the gain
  double fov_y = fov_y_rad_ / M_PI * 180.0f;
  double fov_p = fov_p_rad_ / M_PI * 180.0f;

  double dphi = 10, dtheta = 10;
  double dphi_rad = M_PI * dphi / 180.0f, dtheta_rad = M_PI * dtheta / 180.0f;
  double r;
  int phi, theta;
  double phi_rad, theta_rad;

  //voxblox::Point vec;
  Eigen::Vector3d vec;
  double min_x = static_cast<double>(min_x_);
  double min_y = static_cast<double>(min_y_);
  double min_z = static_cast<double>(min_z_);
  double max_x = static_cast<double>(max_x_);
  double max_y = static_cast<double>(max_y_);
  double max_z = static_cast<double>(max_z_);

  int id = 0;
  for (theta = yaw - fov_y/2; theta < yaw + fov_y/2; theta += dtheta) {
    theta_rad = M_PI * theta / 180.0f;
    for (phi = 90 - fov_p / 2; phi < 90 + fov_p / 2; phi += dphi) {
      phi_rad = M_PI * phi / 180.0f;

      bool occupied_ray = false;
      double g = 0;
      voxblox::Pointcloud voxels_ray;
      for (r = 0; r < r_max_; r += dr_) {
        vec[0] = pose.position_W[0] + r * cos(theta_rad) * sin(phi_rad);
        vec[1] = pose.position_W[1] + r * sin(theta_rad) * sin(phi_rad);
        vec[2] = pose.position_W[2] + r * cos(phi_rad);

        if (vec[0] < min_x || vec[0] > max_x || 
        vec[1] < min_y || vec[1] > max_y || 
        vec[2] < min_z || vec[2] > max_z) {
          continue;
        }

        VoxelStatus node = getVoxelStatus(vec);

        if (node == kOccupied) {
          occupied_ray = true;
          break;
        } else if (node == kFree) {
          continue;
        } else if (node == kUnknown) {
          g += (2 * r * r * dr_ + 1 / 6 * dr_ * dr_ * dr_) * dtheta_rad * sin(phi_rad) * sin(dphi_rad / 2);
          Eigen::Vector3d Voxel;
          getVoxelCenter(&Voxel, vec);
          
          voxblox::Point VoxelCenter;
          VoxelCenter[0] = Voxel[0];
          VoxelCenter[1] = Voxel[1];
          VoxelCenter[2] = Voxel[2];
          voxels_ray.push_back(VoxelCenter);
        }
      }
      voxels.insert(voxels.end(), voxels_ray.begin(), voxels_ray.end());      
      gain += g;
    }
  }
}

double GainEvaluator::computeGain(const eth_mav_msgs::EigenTrajectoryPoint& pose, int modulus) {
  //auto start = std::chrono::high_resolution_clock::now();
  //eth_trajectory_generation::timing::Timer timer_gain("gain_evaluator");

  cam_model_.setBodyPose(voxblox::Transformation(pose.orientation_W_B.cast<float>(), pose.position_W.cast<float>()));

  // Get the boundaries of the current view.
  Eigen::Vector3f aabb_min, aabb_max;
  cam_model_.getAabb(&aabb_min, &aabb_max);

  // Get the center of the camera to raycast to.
  voxblox::Transformation camera_pose = cam_model_.getCameraPose();
  voxblox::Point camera_center = camera_pose.getPosition();

  double num_unknown = 0.0;
  double num_occluded = 0.0;
  double num_free = 0.0;
  double num_occupied = 0.0;

  // Since some complete blocks may be unallocated, just do the dumbest possible
  // thing: iterate over all voxels in the AABB and check if they belong (which
  // should be quite cheap), then look them up.
  
  //double gain = 0.0;
  int checked_voxels = 0;
  int voxel_index = 0;
  Eigen::Vector3f pos = aabb_min.cast<float>();

  const float min_x = std::max(aabb_min.x(), min_x_);
  const float max_x = std::min(aabb_max.x(), max_x_);
  const float min_y = std::max(aabb_min.y(), min_y_);
  const float max_y = std::min(aabb_max.y(), max_y_);
  const float min_z = std::max(aabb_min.z(), min_z_);
  const float max_z = std::min(aabb_max.z(), max_z_);

  for (pos.x() = min_x; pos.x() < max_x; pos.x() += voxel_size_) {
    for (pos.y() = min_y; pos.y() < max_y; pos.y() += voxel_size_) {
      for (pos.z() = min_z; pos.z() < max_z; pos.z() += voxel_size_) {
        if (!cam_model_.isPointInView(pos)) {
          continue;
        }
        if (voxel_index % modulus != 0) {
          voxel_index++;
          continue;
        }
        voxel_index++;
        checked_voxels++;

        // Get the block + voxel index of this voxel by projecting it into
        // the voxel grid and then computing from the global index.
        // This is a truncating cast.
        voxblox::GlobalIndex global_voxel_idx = (voxel_size_inv_ * pos).cast<voxblox::LongIndexElement>();
        voxblox::BlockIndex block_index = voxblox::getBlockIndexFromGlobalVoxelIndex(global_voxel_idx, voxels_per_side_inv_);
        voxblox::VoxelIndex voxel_index = voxblox::getLocalFromGlobalVoxelIndex(global_voxel_idx, voxels_per_side_);

        // Check if this voxel is occluded.
        const voxblox::Point start_scaled = camera_center * voxel_size_inv_;
        const voxblox::Point end_scaled = pos * voxel_size_inv_;

        voxblox::AlignedVector<voxblox::GlobalIndex> global_voxel_indices;
        voxblox::castRay(start_scaled, end_scaled, &global_voxel_indices);

        // Iterate over all the voxels in the index in order.
        // Put them in the checked queue, and classify them. We're starting from
        // the camera center to the current pose, so this defines how we handle
        // occlusions. Don't raycast the last voxel, since it's the actual
        // voxel we're checking (ok if it's occupied, still not an occlusion).
        bool ray_occluded = false;
        for (int i = 0; i < global_voxel_indices.size() - 1; i++) {
          const voxblox::GlobalIndex& global_voxel_idx = global_voxel_indices[i];
          voxblox::BlockIndex block_index_ray = voxblox::getBlockIndexFromGlobalVoxelIndex(global_voxel_idx, voxels_per_side_inv_);
          voxblox::VoxelIndex voxel_index_ray = voxblox::getLocalFromGlobalVoxelIndex(global_voxel_idx, voxels_per_side_);

          // Otherwise look up this voxel and add it to checked.
          const voxblox::Block<voxblox::TsdfVoxel>::Ptr block_ptr = tsdf_layer_->getBlockPtrByIndex(block_index_ray);
          if (block_ptr) {
            // If this block exists, get the voxel.
            const voxblox::TsdfVoxel& voxel = block_ptr->getVoxelByVoxelIndex(voxel_index_ray);
            if (voxel.weight > 1e-6 && voxel.distance <= 0.0) {
              // This is an occupied voxel! Mark all the stuff behind it as occluded.
              ray_occluded = true;
              break;
            }
          }
        }
        if (ray_occluded) {
          num_occluded++;
        } else {
          double distance = 0.0;
          Eigen::Vector3d position = pos.cast<double>();
          if (esdf_map_->getDistanceAtPosition(position, &distance)) {
            if (distance < voxel_size_) {
              continue;
            }
          } else {
            num_unknown++;
          }
        }
      }
    }
  }
  //timer_gain.Stop();

  return num_unknown;
}

void GainEvaluator::computeGainFromsampledYaw(const std::shared_ptr<rrt_star::Node>& node, int yaw_samples, eth_mav_msgs::EigenTrajectoryPoint& trajectory_point) {
    double best_gain = 0;
    double gain;
    double best_yaw;
    for (int k = 0; k < yaw_samples; ++k) {
        double yaw = k * 2 * M_PI / yaw_samples;
        trajectory_point.position_W = node->point.head(3);
        trajectory_point.setFromYaw(yaw);
        gain = computeGain(trajectory_point);
        if (gain > best_gain) {
            best_gain = gain;
            best_yaw = yaw;
        }
    }
    node->gain = best_gain;
    node->point[3] = best_yaw;
}

void GainEvaluator::computeCost(std::shared_ptr<rrt_star::Node>& new_node) {
    new_node->cost = new_node->parent->cost + (new_node->point.head(3) - new_node->parent->point.head(3)).norm();
}

void GainEvaluator::computeScore(std::shared_ptr<rrt_star::Node>& new_node, double lambda) {
    new_node->score = new_node->parent->score + new_node->gain * exp(-lambda * new_node->cost);
}

void GainEvaluator::computeCost(std::shared_ptr<kino_rrt_star::Trajectory>& new_trajectory) {
    new_trajectory->cost = new_trajectory->parent->cost + new_trajectory->cost;
}

void GainEvaluator::computeCostTwo(std::shared_ptr<kino_rrt_star::Trajectory>& new_trajectory) {
    new_trajectory->cost1 = new_trajectory->parent->cost1 + new_trajectory->cost1;
    new_trajectory->cost2 = new_trajectory->parent->cost2 + new_trajectory->cost2;
}

void GainEvaluator::computeScore(std::shared_ptr<kino_rrt_star::Trajectory>& new_trajectory, double lambda) {
    new_trajectory->score = new_trajectory->parent->score + new_trajectory->gain * exp(-lambda * new_trajectory->cost);
}

void GainEvaluator::computeScore(std::shared_ptr<kino_rrt_star::Trajectory>& new_trajectory, double lambda1, double lambda2) {
    new_trajectory->score = new_trajectory->parent->score + new_trajectory->gain * exp(-lambda1 * new_trajectory->cost1 - lambda2 * new_trajectory->cost2);
}

void GainEvaluator::computeSingleScore(std::shared_ptr<kino_rrt_star::Trajectory>& new_trajectory, double lambda) {
    new_trajectory->score = new_trajectory->gain * exp(-lambda * new_trajectory->cost);
}

void GainEvaluator::computeSingleScore(std::shared_ptr<kino_rrt_star::Trajectory>& new_trajectory, double lambda1, double lambda2) {
    new_trajectory->score = new_trajectory->gain * exp(-lambda1 * new_trajectory->cost1 - lambda2 * new_trajectory->cost2);
}

voxblox::CameraModel& GainEvaluator::getCameraModel() { return cam_model_; }

const voxblox::CameraModel& GainEvaluator::getCameraModel() const {
  return cam_model_;
}
