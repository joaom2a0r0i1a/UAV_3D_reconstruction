#include <ros/ros.h>
#include <eth_trajectory_generation/timing.h>
#include <voxblox/integrator/integrator_utils.h>

#include "rrt_construction/gain_evaluator.h"

GainEvaluator::GainEvaluator(const ros::NodeHandle& nh_private) {
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
  nh_private.param("camera_intrinsics/pitch", camera_pitch_, 10.0);

  nh_private.param("accurate_frontiers", p_accurate_frontiers_, false);
  nh_private.param("checking_distance", p_checking_distance_, 2.0);

  nh_private.param("map/voxel_size", dr_, 0.2);

  // initialize neighbor offsets
  auto vs = dr_ * p_checking_distance_;
  if (!p_accurate_frontiers_) {
    c_neighbor_voxels_[0] = Eigen::Vector3d(vs, 0, 0);
    c_neighbor_voxels_[1] = Eigen::Vector3d(-vs, 0, 0);
    c_neighbor_voxels_[2] = Eigen::Vector3d(0, vs, 0);
    c_neighbor_voxels_[3] = Eigen::Vector3d(0, -vs, 0);
    c_neighbor_voxels_[4] = Eigen::Vector3d(0, 0, vs);
    c_neighbor_voxels_[5] = Eigen::Vector3d(0, 0, -vs);
  } else {
    c_neighbor_voxels_[0] = Eigen::Vector3d(vs, 0, 0);
    c_neighbor_voxels_[1] = Eigen::Vector3d(vs, vs, 0);
    c_neighbor_voxels_[2] = Eigen::Vector3d(vs, -vs, 0);
    c_neighbor_voxels_[3] = Eigen::Vector3d(vs, 0, vs);
    c_neighbor_voxels_[4] = Eigen::Vector3d(vs, vs, vs);
    c_neighbor_voxels_[5] = Eigen::Vector3d(vs, -vs, vs);
    c_neighbor_voxels_[6] = Eigen::Vector3d(vs, 0, -vs);
    c_neighbor_voxels_[7] = Eigen::Vector3d(vs, vs, -vs);
    c_neighbor_voxels_[8] = Eigen::Vector3d(vs, -vs, -vs);
    c_neighbor_voxels_[9] = Eigen::Vector3d(0, vs, 0);
    c_neighbor_voxels_[10] = Eigen::Vector3d(0, -vs, 0);
    c_neighbor_voxels_[11] = Eigen::Vector3d(0, 0, vs);
    c_neighbor_voxels_[12] = Eigen::Vector3d(0, vs, vs);
    c_neighbor_voxels_[13] = Eigen::Vector3d(0, -vs, vs);
    c_neighbor_voxels_[14] = Eigen::Vector3d(0, 0, -vs);
    c_neighbor_voxels_[15] = Eigen::Vector3d(0, vs, -vs);
    c_neighbor_voxels_[16] = Eigen::Vector3d(0, -vs, -vs);
    c_neighbor_voxels_[17] = Eigen::Vector3d(-vs, 0, 0);
    c_neighbor_voxels_[18] = Eigen::Vector3d(-vs, vs, 0);
    c_neighbor_voxels_[19] = Eigen::Vector3d(-vs, -vs, 0);
    c_neighbor_voxels_[20] = Eigen::Vector3d(-vs, 0, vs);
    c_neighbor_voxels_[21] = Eigen::Vector3d(-vs, vs, vs);
    c_neighbor_voxels_[22] = Eigen::Vector3d(-vs, -vs, vs);
    c_neighbor_voxels_[23] = Eigen::Vector3d(-vs, 0, -vs);
    c_neighbor_voxels_[24] = Eigen::Vector3d(-vs, vs, -vs);
    c_neighbor_voxels_[25] = Eigen::Vector3d(-vs, -vs, -vs);
  }

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

}

double GainEvaluator::getVerticalFoV(double horizontal_fov, int resolution_x, int resolution_y){
  double aspect_ratio = resolution_x/resolution_y;
  double vertical_fov = 2 * std::atan(std::tan(horizontal_fov / 2) / aspect_ratio);
  return vertical_fov;
}

void GainEvaluator::setCameraModelParametersFoV(double horizontal_fov, double vertical_fov,
                                                double min_distance, double max_distance) {
  cam_model_.setIntrinsicsFromFoV(horizontal_fov, vertical_fov, min_distance, max_distance);
  prev_cam_model_.setIntrinsicsFromFoV(horizontal_fov, vertical_fov, min_distance, max_distance);
}

void GainEvaluator::setCameraModelParametersFocalLength(
    const Eigen::Vector2d& resolution, double focal_length, double min_distance,
    double max_distance) {
  cam_model_.setIntrinsicsFromFocalLength(
      resolution.cast<float>(), focal_length, min_distance, max_distance);
  prev_cam_model_.setIntrinsicsFromFocalLength(
      resolution.cast<float>(), focal_length, min_distance, max_distance);
}

void GainEvaluator::setCameraExtrinsics(const voxblox::Transformation& T_C_B) {
  cam_model_.setExtrinsics(T_C_B);
  prev_cam_model_.setExtrinsics(T_C_B);
}

bool GainEvaluator::isPointInView(const voxblox::Point& point, bool first_node) const {
  if (first_node) return false;
  for (size_t i = 0; i < bounding_planes_.size(); i++) {
    if (!bounding_planes_[i].isPointInside(point)) {
      return false;
    }
  }
  return true;
}

std::pair<double, double> GainEvaluator::rayFrustumIntersectionSegment(const voxblox::Point& ray_origin, const voxblox::Point& ray_dir, double min_range, double max_range) const {
  voxblox::Point dir = ray_dir.normalized();
  double t_enter = -std::numeric_limits<double>::infinity();
  double t_exit  =  std::numeric_limits<double>::infinity();

  for (const voxblox::Plane& plane : bounding_planes_) {
    const auto& n = plane.normal();
    double denom = n.dot(dir);
    double numerator = plane.distance() - n.dot(ray_origin);

    if (std::abs(denom) < 1e-9) {
      // Ray is parallel to plane
      if (numerator > 0.0) {
        // Ray is outside the frustum
        return std::make_pair(min_range, 0.0);
      }
      // Ray is parallel and inside or on the plane
      continue;
    }

    double t_hit = numerator / denom;

    if (denom > 0.0) {
      // Ray entering the plane
      t_enter = std::max(t_enter, t_hit);
    } else {
      // Ray exiting the plane
      t_exit = std::min(t_exit, t_hit);
    }

    if (t_enter > t_exit) {
      // No intersection
      return std::make_pair(min_range, 0.0);
    }
  }

  if (t_exit < 0.0) {
    // Intersection is behind the ray origin
    return std::make_pair(min_range, 0.0);
  }

  // Clamp to ray range
  t_enter = std::max(t_enter, min_range);
  t_exit  = std::min(t_exit,  max_range);

  double length_inside = t_exit - t_enter;
  if (length_inside <= 0.0) {
    return std::make_pair(min_range, 0.0);
  }

  return std::make_pair(t_enter, length_inside);
}

std::vector<std::pair<double, double>> GainEvaluator::rayMultiFrustumIntersectionSegment(const voxblox::Point& ray_origin, const voxblox::Point& ray_dir, double min_range, double max_range) const {
  voxblox::Point dir = ray_dir.normalized();
  std::vector<std::pair<double, double>> intervals;

  // Loop through each frustum
  for (const voxblox::AlignedVector<voxblox::Plane>& planes : frustums) {
      double t_enter = -std::numeric_limits<double>::infinity();
      double t_exit  =  std::numeric_limits<double>::infinity();

      bool valid = true;

      for (const voxblox::Plane& plane : planes) {
          const auto& n = plane.normal();
          double denom = n.dot(dir);
          double numerator = plane.distance() - n.dot(ray_origin);

          if (std::abs(denom) < 1e-9) {
              if (numerator > 0.0) {
                  valid = false;
                  break;
              }
              continue;
          }

          double t_hit = numerator / denom;

          if (denom > 0.0) {
              t_enter = std::max(t_enter, t_hit);
          } else {
              t_exit = std::min(t_exit, t_hit);
          }

          //t_enter = (denom < 0.0) ? std::max(t_enter, t_hit) : t_enter;
          //t_exit  = (denom > 0.0) ? std::min(t_exit, t_hit) : t_exit;

          if (t_enter >= t_exit) {
              valid = false;
              break;
          }
      }

      if (!valid || t_exit <= 0.0) { 
        continue;
      }

      // Clamp to ray range
      t_enter = std::max(t_enter, min_range);
      t_exit  = std::min(t_exit, max_range);

      if (t_exit > t_enter) {
          intervals.push_back({t_enter, t_exit});
      }
  }

  if (intervals.empty() || intervals.size() == 1) {
    return intervals;
  }

  /*// Merge overlapping intervals
  std::vector<std::pair<double, double>> merged;
  merged.push_back(intervals[0]);
  for (size_t i = 1; i < intervals.size(); ++i) {
    auto& last = merged.back();
    if (intervals[i].first > last.second) {
        merged.push_back(intervals[i]);
    } else {
        last.second = std::max(last.second, intervals[i].second);
    }
  }*/

  // Sort by start
  std::sort(intervals.begin(), intervals.end(),
            [](const std::pair<double, double>& a, const std::pair<double, double>& b) {
                return a.first < b.first;
            });

  // Merge overlapping intervals
  std::vector<std::pair<double, double>> merged;
  merged.reserve(intervals.size());
  for (const auto& iv : intervals) {
      if (merged.empty() || iv.first > merged.back().second) {
          merged.push_back(iv);
      } else {
          merged.back().second = std::max(merged.back().second, iv.second);
      }
  }

  return merged;
}

bool GainEvaluator::isFrontierVoxel(const Eigen::Vector3d& voxel) {
  // Check all neighboring voxels
  VoxelStatus voxel_state;
  if (!p_accurate_frontiers_) {
    for (int i = 0; i < 6; ++i) {
      voxel_state = getVoxelStatus(voxel + c_neighbor_voxels_[i]);
      if (voxel_state == kUnknown) {
        continue;
      }
      return voxel_state == kOccupied;
    }
  } else {
    for (int i = 0; i < 26; ++i) {
      voxel_state = getVoxelStatus(voxel + c_neighbor_voxels_[i]);
      if (voxel_state == kUnknown) {
        continue;
      }
      return voxel_state == kOccupied;
    }
  }
  return false;
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
  if (voxel->distance > voxel_size_) {
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
    for (phi = 90 - fov_p / 2 + camera_pitch_; phi < 90 + fov_p / 2 + camera_pitch_; phi += dphi) {
      phi_rad = M_PI * phi / 180.0f;

      double g = 0;
      bool occupied_ray = false;
      for (r = 0; r < r_max_; r += dr_) {
        vec[0] = camera_center.x() + r * cos(theta_rad) * sin(phi_rad);
        vec[1] = camera_center.y() + r * sin(theta_rad) * sin(phi_rad);
        vec[2] = camera_center.z() + r * cos(phi_rad);

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
    for (phi = 90 - fov_p / 2 + camera_pitch_; phi < 90 + fov_p / 2 + camera_pitch_; phi += dphi) {
      phi_rad = M_PI * phi / 180.0f;

      double g = 0;
      bool occupied_ray = false;
      for (r = 0; r < r_max_; r += dr_) {
        vec[0] = camera_center.x() + r * cos(theta_rad) * sin(phi_rad);
        vec[1] = camera_center.y() + r * sin(theta_rad) * sin(phi_rad);
        vec[2] = camera_center.z() + r * cos(phi_rad);

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
    for (phi = 90 - fov_p / 2 + camera_pitch_; phi < 90 + fov_p / 2 + camera_pitch_; phi += dphi) {
      phi_rad = M_PI * phi / 180.0f;

      double g = 0;
      bool occupied_ray = false;
      for (r = 0; r < r_max_; r += dr_) {
        vec[0] = camera_center.x() + r * cos(theta_rad) * sin(phi_rad);
        vec[1] = camera_center.y() + r * sin(theta_rad) * sin(phi_rad);
        vec[2] = camera_center.z() + r * cos(phi_rad);

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
  double yaw_step = 2 * M_PI / yaw_samples_;
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

double GainEvaluator::computeGainFixedAngleAEP(const eth_mav_msgs::EigenTrajectoryPoint& previous_pose, const eth_mav_msgs::EigenTrajectoryPoint& pose, bool first_node, int modulus) {
  CHECK_NOTNULL(tsdf_layer_);

  //auto start = std::chrono::high_resolution_clock::now();

  // Set the camera pose to the previous one to get the previous Frustum
  prev_cam_model_.setBodyPose(voxblox::Transformation(
      previous_pose.orientation_W_B.cast<float>(), previous_pose.position_W.cast<float>()));

  bounding_planes_ = prev_cam_model_.getBoundingPlanes();

  // Now set camera model to the current pose
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

  voxblox::Point vec_voxblox;
  Eigen::Vector3d vec;
  
  double min_x = static_cast<double>(min_x_);
  double min_y = static_cast<double>(min_y_);
  double min_z = static_cast<double>(min_z_);
  double max_x = static_cast<double>(max_x_);
  double max_y = static_cast<double>(max_y_);
  double max_z = static_cast<double>(max_z_);

  double start_dist, intersect_len;

  int id = 0;
  for (theta = yaw - fov_y/2; theta < yaw + fov_y/2; theta += dtheta) {
    theta_rad = M_PI * theta / 180.0f;
    for (phi = 90 - fov_p / 2 + camera_pitch_; phi < 90 + fov_p / 2 + camera_pitch_; phi += dphi) {
      phi_rad = M_PI * phi / 180.0f;
      double g = 0.0;

      voxblox::Point ray_dir(cos(theta_rad) * sin(phi_rad), sin(theta_rad) * sin(phi_rad), cos(phi_rad));
      std::pair<double, double> intersect_segment = rayFrustumIntersectionSegment(camera_center, ray_dir, dr_, r_max_);

      double start_dist = intersect_segment.first;
      double intersect_len = intersect_segment.second;

      for (r = dr_; r < r_max_; r += dr_) {
        if (r >= start_dist && intersect_len > 0.0) {
          r += intersect_len;
          if (r >= r_max_) {
            break;
          }
        }

        vec[0] = camera_center.x() + r * cos(theta_rad) * sin(phi_rad);
        vec[1] = camera_center.y() + r * sin(theta_rad) * sin(phi_rad);
        vec[2] = camera_center.z() + r * cos(phi_rad);
        
        vec_voxblox = vec.cast<float>();

        /*if (isPointInView(vec_voxblox, first_node)) {
          continue;
        }*/

        /*if (isInsideAABB(vec, aabb_min, aabb_max, first_node)) {
          break;
        }*/

        if (vec[0] < min_x || vec[0] > max_x || 
        vec[1] < min_y || vec[1] > max_y || 
        vec[2] < min_z || vec[2] > max_z) {
          continue;
        }

        VoxelStatus node = getVoxelStatus(vec);

        if (node == kOccupied) {
          break;
        } else if (node == kFree) {
          continue;
        } else if (node == kUnknown) {
          g += (2 * r * r * dr_ + 1 / 6 * dr_ * dr_ * dr_) * dtheta_rad * sin(phi_rad) * sin(dphi_rad / 2);
          /*if (isFrontierVoxel(vec)) {
            g += (2 * r * r * dr_ + 1 / 6 * dr_ * dr_ * dr_) * dtheta_rad * sin(phi_rad) * sin(dphi_rad / 2) / (voxel_size_ * voxel_size_ * voxel_size_);
          }*/
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
    for (phi = 90 - fov_p / 2 + camera_pitch_; phi < 90 + fov_p / 2 + camera_pitch_; phi += dphi) {
      phi_rad = M_PI * phi / 180.0f;

      double g = 0.0;
      bool occupied_ray = false;
      for (r = 0; r < r_max_; r += dr_) {
        vec[0] = camera_center.x() + r * cos(theta_rad) * sin(phi_rad);
        vec[1] = camera_center.y() + r * sin(theta_rad) * sin(phi_rad);
        vec[2] = camera_center.z() + r * cos(phi_rad);

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
          /*if (isFrontierVoxel(vec)) {
            g += (2 * r * r * dr_ + 1 / 6 * dr_ * dr_ * dr_) * dtheta_rad * sin(phi_rad) * sin(dphi_rad / 2) / (voxel_size_ * voxel_size_ * voxel_size_);
          }*/
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
    for (phi = 90 - fov_p / 2 + camera_pitch_; phi < 90 + fov_p / 2 + camera_pitch_; phi += dphi) {
      phi_rad = M_PI * phi / 180.0f;

      double g = 0.0;
      bool occupied_ray = false;
      for (r = 0; r < r_max_; r += dr_) {
        vec[0] = camera_center.x() + r * cos(theta_rad) * sin(phi_rad);
        vec[1] = camera_center.y() + r * sin(theta_rad) * sin(phi_rad);
        vec[2] = camera_center.z() + r * cos(phi_rad);

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
    for (phi = 90 - fov_p / 2 + camera_pitch_; phi < 90 + fov_p / 2 + camera_pitch_; phi += dphi) {
      phi_rad = M_PI * phi / 180.0f;

      double g = 0;
      bool occupied_ray = false;
      for (r = 0; r < r_max_; r += dr_) {
        vec[0] = camera_center.x() + r * cos(theta_rad) * sin(phi_rad);
        vec[1] = camera_center.y() + r * sin(theta_rad) * sin(phi_rad);
        vec[2] = camera_center.z() + r * cos(phi_rad);

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
    for (phi = 90 - fov_p / 2 + camera_pitch_; phi < 90 + fov_p / 2 + camera_pitch_; phi += dphi) {
      phi_rad = M_PI * phi / 180.0f;

      double g = 0;
      bool occupied_ray = false;
      for (r = 0; r < r_max_; r += dr_) {
        vec[0] = camera_center.x() + r * cos(theta_rad) * sin(phi_rad);
        vec[1] = camera_center.y() + r * sin(theta_rad) * sin(phi_rad);
        vec[2] = camera_center.z() + r * cos(phi_rad);

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
          /*if (isFrontierVoxel(vec)) {
            g += (2 * r * r * dr_ + 1 / 6 * dr_ * dr_ * dr_) * dtheta_rad * sin(phi_rad) * sin(dphi_rad / 2) / (voxel_size_ * voxel_size_ * voxel_size_);
          }*/
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
  double yaw_step = 2 * M_PI / yaw_samples_;
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

std::tuple<double, double, double> GainEvaluator::computeGainOptimizedGEO(const eth_mav_msgs::EigenTrajectoryPoint& pose, int modulus) {
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
  double max_gain = 0.0;

  // This function computes the gain
  double fov_y = fov_y_rad_ / M_PI * 180.0f;
  double fov_p = fov_p_rad_ / M_PI * 180.0f;

  double dphi = 10, dtheta = 10;
  double dphi_rad = M_PI * dphi / 180.0f, dtheta_rad = M_PI * dtheta / 180.0f;
  double r;
  int phi, theta;
  double phi_rad, theta_rad;

  std::map<int, double> gain_per_yaw;
  std::map<int, double> max_gain_per_yaw;

  //voxblox::Point vec;
  Eigen::Vector3d vec;
  double min_x = static_cast<double>(min_x_);
  double min_y = static_cast<double>(min_y_);
  double min_z = static_cast<double>(min_z_);
  double max_x = static_cast<double>(max_x_);
  double max_y = static_cast<double>(max_y_);
  double max_z = static_cast<double>(max_z_);

  bool has_frontier = false;

  int id = 0;
  for (theta = -180; theta < 180; theta += dtheta) {
    theta_rad = M_PI * theta / 180.0f;
    for (phi = 90 - fov_p / 2 + camera_pitch_; phi < 90 + fov_p / 2 + camera_pitch_; phi += dphi) {
      phi_rad = M_PI * phi / 180.0f;
      double g = 0;
      double mg = 0;

      mg = ((pow(r_max_, 3) - pow(dr_, 3)) / 3.0) * dtheta_rad * sin(phi_rad) * dphi_rad;
      max_gain += mg;
      max_gain_per_yaw[theta] += mg;

      for (r = 0; r < r_max_; r += dr_) {
        vec[0] = camera_center.x() + r * cos(theta_rad) * sin(phi_rad);
        vec[1] = camera_center.y() + r * sin(theta_rad) * sin(phi_rad);
        vec[2] = camera_center.z() + r * cos(phi_rad);

        /*mg += (2 * r * r * dr_ + 1 / 6 * dr_ * dr_ * dr_) * dtheta_rad * sin(phi_rad) * sin(dphi_rad / 2);
        max_gain += mg;
        max_gain_per_yaw[theta] += mg;*/

        if (vec[0] < min_x || vec[0] > max_x || 
        vec[1] < min_y || vec[1] > max_y || 
        vec[2] < min_z || vec[2] > max_z) {
          continue;
        }

        VoxelStatus node = getVoxelStatus(vec);

        if (node == kOccupied) {
          break;
        } else if (node == kFree) {
          continue;
        } else if (node == kUnknown) {
          g += (2 * r * r * dr_ + 1 / 6 * dr_ * dr_ * dr_) * dtheta_rad * sin(phi_rad) * sin(dphi_rad / 2);
          if (isFrontierVoxel(vec)) {
            has_frontier = true;
            //g += (2 * r * r * dr_ + 1 / 6 * dr_ * dr_ * dr_) * dtheta_rad * sin(phi_rad) * sin(dphi_rad / 2) / (voxel_size_ * voxel_size_ * voxel_size_);
          }
        }
      }
      gain += g;
      gain_per_yaw[theta] += g;
    }
  }

  if (!has_frontier) {
    return std::make_tuple(0.0, 0.0, 0.0);
  }

  //auto intermediate = std::chrono::high_resolution_clock::now();

  double best_gain = 0;
  double best_yaw = 0;
  double best_max_gain = 0;

  int min_yaw_samples = ceil(2 * M_PI / fov_y_rad_);

  std::vector<double> yaws;
  std::vector<double> gains;
  double min_yaw_step = 2 * M_PI / min_yaw_samples;
  double yaw_step = 2 * M_PI / yaw_samples_;
  int aditional_angles = (360 - min_yaw_samples) / min_yaw_samples;

  for (int k = 0; k < min_yaw_samples; ++k) {
    double yaw_optimized = k * min_yaw_step / M_PI * 180.0f;
    double gain_optimized = 0;
    double max_gain_optimized = 0;
    for (int fov = -fov_y / 2; fov < fov_y / 2; fov++) {
      int theta = yaw_optimized + fov;
      if (theta < -180)
        theta += 360;
      if (theta > 180)
        theta -= 360;
      gain_optimized += gain_per_yaw[theta];
      max_gain_optimized += max_gain_per_yaw[theta];
    }

    yaws.push_back(yaw_optimized);
    gains.push_back(gain_optimized);

    if (gain_optimized > best_gain) {
      best_gain = gain_optimized;
      best_max_gain = max_gain_optimized;
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
      double max_gain_optimized = 0;
      for (int fov = -fov_y / 2; fov < fov_y / 2; fov++) {
        int theta = yaw_optimized + fov;
        if (theta < -180)
          theta += 360;
        if (theta > 180)
          theta -= 360;
        gain_optimized += gain_per_yaw[theta];
        max_gain_optimized += max_gain_per_yaw[theta];
      }

      if (gain_optimized > best_gain) {
        best_gain = gain_optimized;
        best_max_gain = max_gain_optimized;
        best_yaw = yaw_optimized;
      }
    }
  }

  gain = best_gain;
  max_gain = best_max_gain;
  yaw = M_PI * best_yaw / 180.f;

  //auto end = std::chrono::high_resolution_clock::now();
  //std::chrono::duration<double> elapsed = end - start;
  //std::chrono::duration<double> intermediate_elapsed = intermediate - start;
  //std::cout << "First Part AEPGain took " << intermediate_elapsed.count() << " seconds." << std::endl;
  //std::cout << "AEPGain took " << elapsed.count() << " seconds." << std::endl;

  return std::make_tuple(gain, max_gain, yaw);
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
    for (phi = 90 - fov_p / 2 + camera_pitch_; phi < 90 + fov_p / 2 + camera_pitch_; phi += dphi) {
      phi_rad = M_PI * phi / 180.0f ;

      double g = 0;
      bool occupied_ray = false;
      for (r = 0; r < r_max_; r += dr_) {
        vec[0] = camera_center.x() + r * cos(theta_rad) * sin(phi_rad);
        vec[1] = camera_center.y() + r * sin(theta_rad) * sin(phi_rad);
        vec[2] = camera_center.z() + r * cos(phi_rad);

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
  double yaw_step = 2 * M_PI / yaw_samples_;
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

std::pair<double, double> GainEvaluator::computeGainOptimizedwithPrior(const eth_mav_msgs::EigenTrajectoryPoint& previous_pose, const eth_mav_msgs::EigenTrajectoryPoint& pose) {
  CHECK_NOTNULL(tsdf_layer_);

  //auto start = std::chrono::high_resolution_clock::now();

  // Set the camera pose to the previous one to get the previous Frustum
  prev_cam_model_.setBodyPose(voxblox::Transformation(
      previous_pose.orientation_W_B.cast<float>(), previous_pose.position_W.cast<float>()));

  bounding_planes_ = prev_cam_model_.getBoundingPlanes();

  // Now set camera model to the current pose
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

  std::map<int, double> gain_per_yaw;

  voxblox::Point vec_voxblox;
  Eigen::Vector3d vec;
  
  double min_x = static_cast<double>(min_x_);
  double min_y = static_cast<double>(min_y_);
  double min_z = static_cast<double>(min_z_);
  double max_x = static_cast<double>(max_x_);
  double max_y = static_cast<double>(max_y_);
  double max_z = static_cast<double>(max_z_);

  double start_dist, intersect_len;

  int id = 0;
  for (theta = -180; theta < 180; theta += dtheta) {
    theta_rad = M_PI * theta / 180.0f;
    for (phi = 90 - fov_p / 2 + camera_pitch_; phi < 90 + fov_p / 2 + camera_pitch_; phi += dphi) {
      phi_rad = M_PI * phi / 180.0f;
      double g = 0.0;

      voxblox::Point ray_dir(cos(theta_rad) * sin(phi_rad), sin(theta_rad) * sin(phi_rad), cos(phi_rad));
      std::pair<double, double> intersect_segment = rayFrustumIntersectionSegment(camera_center, ray_dir, dr_, r_max_);

      double start_dist = intersect_segment.first;
      double intersect_len = intersect_segment.second;

      for (r = dr_; r < r_max_; r += dr_) {
        if (r >= start_dist && intersect_len > 0.0) {
          r += intersect_len;
          if (r >= r_max_) {
            break;
          }
        }

        vec[0] = camera_center.x() + r * cos(theta_rad) * sin(phi_rad);
        vec[1] = camera_center.y() + r * sin(theta_rad) * sin(phi_rad);
        vec[2] = camera_center.z() + r * cos(phi_rad);
        
        vec_voxblox = vec.cast<float>();

        /*if (isPointInView(vec_voxblox, first_node)) {
          continue;
        }*/

        /*if (isInsideAABB(vec, aabb_min, aabb_max, first_node)) {
          break;
        }*/

        if (vec[0] < min_x || vec[0] > max_x || 
        vec[1] < min_y || vec[1] > max_y || 
        vec[2] < min_z || vec[2] > max_z) {
          continue;
        }

        VoxelStatus node = getVoxelStatus(vec);

        if (node == kOccupied) {
          break;
        } else if (node == kFree) {
          continue;
        } else if (node == kUnknown) {
          g += (2 * r * r * dr_ + 1 / 6 * dr_ * dr_ * dr_) * dtheta_rad * sin(phi_rad) * sin(dphi_rad / 2);
          /*if (isFrontierVoxel(vec)) {
            g += (2 * r * r * dr_ + 1 / 6 * dr_ * dr_ * dr_) * dtheta_rad * sin(phi_rad) * sin(dphi_rad / 2) / (voxel_size_ * voxel_size_ * voxel_size_);
          }*/
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
  double yaw_step = 2 * M_PI / yaw_samples_;
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
  yaw = M_PI * best_yaw / 180.f;

  //auto end = std::chrono::high_resolution_clock::now();
  //std::chrono::duration<double> elapsed = end - start;
  //std::chrono::duration<double> intermediate_elapsed = intermediate - start;
  //std::cout << "First Part AEPGain took " << intermediate_elapsed.count() << " seconds." << std::endl;
  //std::cout << "AEPGain took " << elapsed.count() << " seconds." << std::endl;

  return std::make_pair(gain, yaw);
}

std::pair<double, double> GainEvaluator::computeGainOptimizedwithMultiplePriors(const std::vector<eth_mav_msgs::EigenTrajectoryPoint>& previous_poses, const eth_mav_msgs::EigenTrajectoryPoint& pose) {
  CHECK_NOTNULL(tsdf_layer_);

  //auto start = std::chrono::high_resolution_clock::now();
  frustums.clear();
  frustums.reserve(previous_poses.size());
  for (int i = 0; i < previous_poses.size(); ++i) {
    // Set the camera pose to the previous one to get the previous Frustum
    prev_cam_model_.setBodyPose(voxblox::Transformation(
        previous_poses[i].orientation_W_B.cast<float>(), previous_poses[i].position_W.cast<float>()));

    frustums.push_back(prev_cam_model_.getBoundingPlanes());
  }
  
  /*// Set the camera pose to the previous one to get the previous Frustum
  prev_cam_model_.setBodyPose(voxblox::Transformation(
      previous_pose.orientation_W_B.cast<float>(), previous_pose.position_W.cast<float>()));

  bounding_planes_ = prev_cam_model_.getBoundingPlanes();*/

  // Now set camera model to the current pose
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

  std::map<int, double> gain_per_yaw;

  voxblox::Point vec_voxblox;
  Eigen::Vector3d vec;
  
  double min_x = static_cast<double>(min_x_);
  double min_y = static_cast<double>(min_y_);
  double min_z = static_cast<double>(min_z_);
  double max_x = static_cast<double>(max_x_);
  double max_y = static_cast<double>(max_y_);
  double max_z = static_cast<double>(max_z_);

  double start_dist, intersect_len;

  int id = 0;
  for (theta = -180; theta < 180; theta += dtheta) {
    theta_rad = M_PI * theta / 180.0f;
    for (phi = 90 - fov_p / 2 + camera_pitch_; phi < 90 + fov_p / 2 + camera_pitch_; phi += dphi) {
      phi_rad = M_PI * phi / 180.0f;
      double g = 0.0;

      voxblox::Point ray_dir(cos(theta_rad) * sin(phi_rad), sin(theta_rad) * sin(phi_rad), cos(phi_rad));
      std::vector<std::pair<double, double>> skip_intervals = rayMultiFrustumIntersectionSegment(camera_center, ray_dir, dr_, r_max_);

      size_t skip_idx = 0;
      for (r = dr_; r < r_max_; r += dr_) {
        if (skip_idx < skip_intervals.size() && r >= skip_intervals[skip_idx].first) {
          double r0 = skip_intervals[skip_idx].first;
          double r1 = skip_intervals[skip_idx].second;

          bool occupied_found = false;

          for (double rr = r0; rr < r1; rr += dr_) {
            vec[0] = camera_center.x() + rr * cos(theta_rad) * sin(phi_rad);
            vec[1] = camera_center.y() + rr * sin(theta_rad) * sin(phi_rad);
            vec[2] = camera_center.z() + rr * cos(phi_rad);

            // bounds check
            if (vec[0] < min_x || vec[0] > max_x ||
                vec[1] < min_y || vec[1] > max_y ||
                vec[2] < min_z || vec[2] > max_z) {
              continue;
            }

            VoxelStatus node = getVoxelStatus(vec);

            if (node == kOccupied) {
              occupied_found = true;
              break;
            }
          }

          if (occupied_found) {
            // Surface found inside skipped interval — stop this ray entirely
            break;
          }

          // Otherwise, safe to skip ahead
          r = r1;
          ++skip_idx;

          if (r >= r_max_) {
            break;
          }
          continue;
        }
      /*for (r = dr_; r < r_max_; r += dr_) {
        if (skip_idx < skip_intervals.size() && r >= skip_intervals[skip_idx].first) {
          r = skip_intervals[skip_idx].second;
          ++skip_idx;
          if (r >= r_max_) {
            break;
          }
          continue;
        }*/

        vec[0] = camera_center.x() + r * cos(theta_rad) * sin(phi_rad);
        vec[1] = camera_center.y() + r * sin(theta_rad) * sin(phi_rad);
        vec[2] = camera_center.z() + r * cos(phi_rad);
        
        vec_voxblox = vec.cast<float>();

        /*if (isPointInView(vec_voxblox, first_node)) {
          continue;
        }*/

        /*if (isInsideAABB(vec, aabb_min, aabb_max, first_node)) {
          break;
        }*/

        if (vec[0] < min_x || vec[0] > max_x || 
        vec[1] < min_y || vec[1] > max_y || 
        vec[2] < min_z || vec[2] > max_z) {
          continue;
        }

        VoxelStatus node = getVoxelStatus(vec);

        if (node == kOccupied) {
          break;
        } else if (node == kFree) {
          continue;
        } else if (node == kUnknown) {
          g += (2 * r * r * dr_ + 1 / 6 * dr_ * dr_ * dr_) * dtheta_rad * sin(phi_rad) * sin(dphi_rad / 2);
          /*if (isFrontierVoxel(vec)) {
            g += (2 * r * r * dr_ + 1 / 6 * dr_ * dr_ * dr_) * dtheta_rad * sin(phi_rad) * sin(dphi_rad / 2) / (voxel_size_ * voxel_size_ * voxel_size_);
          }*/
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
  double yaw_step = 2 * M_PI / yaw_samples_;
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
  yaw = M_PI * best_yaw / 180.f;

  //auto end = std::chrono::high_resolution_clock::now();
  //std::chrono::duration<double> elapsed = end - start;
  //std::chrono::duration<double> intermediate_elapsed = intermediate - start;
  //std::cout << "First Part AEPGain took " << intermediate_elapsed.count() << " seconds." << std::endl;
  //std::cout << "AEPGain took " << elapsed.count() << " seconds." << std::endl;

  return std::make_pair(gain, yaw);
}

std::tuple<double, double, double> GainEvaluator::computeGainOptimizedwithMultiplePriorsMax(const std::vector<eth_mav_msgs::EigenTrajectoryPoint>& previous_poses, const eth_mav_msgs::EigenTrajectoryPoint& pose) {
  CHECK_NOTNULL(tsdf_layer_);

  //auto start = std::chrono::high_resolution_clock::now();
  frustums.clear();
  frustums.reserve(previous_poses.size());
  for (int i = 0; i < previous_poses.size(); ++i) {
    // Set the camera pose to the previous one to get the previous Frustum
    prev_cam_model_.setBodyPose(voxblox::Transformation(
        previous_poses[i].orientation_W_B.cast<float>(), previous_poses[i].position_W.cast<float>()));

    frustums.push_back(prev_cam_model_.getBoundingPlanes());
  }
  
  /*// Set the camera pose to the previous one to get the previous Frustum
  prev_cam_model_.setBodyPose(voxblox::Transformation(
      previous_pose.orientation_W_B.cast<float>(), previous_pose.position_W.cast<float>()));

  bounding_planes_ = prev_cam_model_.getBoundingPlanes();*/

  // Now set camera model to the current pose
  cam_model_.setBodyPose(voxblox::Transformation(
      pose.orientation_W_B.cast<float>(), pose.position_W.cast<float>()));

  // Get the center of the camera to raycast to.
  voxblox::Transformation camera_pose = cam_model_.getCameraPose();
  voxblox::Point camera_center = camera_pose.getPosition();

  double yaw_rad = pose.getYaw();
  double yaw = yaw_rad * 180 / M_PI;
  double gain = 0.0;
  double max_gain = 0.0;

  // This function computes the gain
  double fov_y = fov_y_rad_ / M_PI * 180.0f;
  double fov_p = fov_p_rad_ / M_PI * 180.0f;

  double dphi = 10, dtheta = 10;
  double dphi_rad = M_PI * dphi / 180.0f, dtheta_rad = M_PI * dtheta / 180.0f;
  double r;
  int phi, theta;
  double phi_rad, theta_rad;

  std::map<int, double> gain_per_yaw;
  std::map<int, double> max_gain_per_yaw;

  voxblox::Point vec_voxblox;
  Eigen::Vector3d vec;
  
  double min_x = static_cast<double>(min_x_);
  double min_y = static_cast<double>(min_y_);
  double min_z = static_cast<double>(min_z_);
  double max_x = static_cast<double>(max_x_);
  double max_y = static_cast<double>(max_y_);
  double max_z = static_cast<double>(max_z_);

  double start_dist, intersect_len;

  int id = 0;
  for (theta = -180; theta < 180; theta += dtheta) {
    theta_rad = M_PI * theta / 180.0f;
    for (phi = 90 - fov_p / 2 + camera_pitch_; phi < 90 + fov_p / 2 + camera_pitch_; phi += dphi) {
      phi_rad = M_PI * phi / 180.0f;
      double g = 0.0;
      double mg = 0.0;

      voxblox::Point ray_dir(cos(theta_rad) * sin(phi_rad), sin(theta_rad) * sin(phi_rad), cos(phi_rad));
      std::vector<std::pair<double, double>> skip_intervals = rayMultiFrustumIntersectionSegment(camera_center, ray_dir, dr_, r_max_);

      mg = ((pow(r_max_, 3) - pow(dr_, 3)) / 3.0) * dtheta_rad * sin(phi_rad) * dphi_rad;
      max_gain += mg;
      max_gain_per_yaw[theta] += mg;

      for (const auto& interval : skip_intervals) {
        double r0 = std::max(interval.first, dr_);
        double r1 = std::min(interval.second, r_max_);
        if (r0 < r1) {
          double mg_overlap = ((pow(r1, 3) - pow(r0, 3)) / 3.0) *
                              dtheta_rad * sin(phi_rad) * dphi_rad;
          max_gain -= mg_overlap;
          max_gain_per_yaw[theta] -= mg_overlap;
        }
      }

      size_t skip_idx = 0;
      for (r = dr_; r < r_max_; r += dr_) {
        if (skip_idx < skip_intervals.size() && r >= skip_intervals[skip_idx].first) {
          double r0 = skip_intervals[skip_idx].first;
          double r1 = skip_intervals[skip_idx].second;

          bool occupied_found = false;

          for (double rr = r0; rr < r1; rr += dr_) {
            vec[0] = camera_center.x() + rr * cos(theta_rad) * sin(phi_rad);
            vec[1] = camera_center.y() + rr * sin(theta_rad) * sin(phi_rad);
            vec[2] = camera_center.z() + rr * cos(phi_rad);

            // bounds check
            if (vec[0] < min_x || vec[0] > max_x ||
                vec[1] < min_y || vec[1] > max_y ||
                vec[2] < min_z || vec[2] > max_z) {
              continue;
            }

            VoxelStatus node = getVoxelStatus(vec);

            if (node == kOccupied) {
              occupied_found = true;
              break;
            }
          }

          if (occupied_found) {
            // Surface found inside skipped interval — stop this ray entirely
            break;
          }

          // Otherwise, safe to skip ahead
          r = r1;
          ++skip_idx;

          if (r >= r_max_) {
            break;
          }
          continue;
        }
      /*for (r = dr_; r < r_max_; r += dr_) {
        if (skip_idx < skip_intervals.size() && r >= skip_intervals[skip_idx].first) {
          r = skip_intervals[skip_idx].second;
          ++skip_idx;
          if (r >= r_max_) {
            break;
          }
          continue;
        }*/

        vec[0] = camera_center.x() + r * cos(theta_rad) * sin(phi_rad);
        vec[1] = camera_center.y() + r * sin(theta_rad) * sin(phi_rad);
        vec[2] = camera_center.z() + r * cos(phi_rad);
        
        vec_voxblox = vec.cast<float>();

        /*mg += (2 * r * r * dr_ + 1 / 6 * dr_ * dr_ * dr_) * dtheta_rad * sin(phi_rad) * sin(dphi_rad / 2);
        max_gain += mg;
        max_gain_per_yaw[theta] += mg;*/

        /*if (isPointInView(vec_voxblox, first_node)) {
          continue;
        }*/

        /*if (isInsideAABB(vec, aabb_min, aabb_max, first_node)) {
          break;
        }*/

        if (vec[0] < min_x || vec[0] > max_x || 
            vec[1] < min_y || vec[1] > max_y || 
            vec[2] < min_z || vec[2] > max_z) {
          continue;
        }

        VoxelStatus node = getVoxelStatus(vec);

        if (node == kOccupied) {
          break;
        } else if (node == kFree) {
          continue;
        } else if (node == kUnknown) {
          g += (2 * r * r * dr_ + 1 / 6 * dr_ * dr_ * dr_) * dtheta_rad * sin(phi_rad) * sin(dphi_rad / 2);
          /*if (isFrontierVoxel(vec)) {
            g += (2 * r * r * dr_ + 1 / 6 * dr_ * dr_ * dr_) * dtheta_rad * sin(phi_rad) * sin(dphi_rad / 2) / (voxel_size_ * voxel_size_ * voxel_size_);
          }*/
        }
      }
      gain += g;
      gain_per_yaw[theta] += g;
    }
  }
  
  double best_gain = 0;
  double best_yaw = 0;
  double best_max_gain = 0;

  int min_yaw_samples = ceil(2 * M_PI / fov_y_rad_);

  std::vector<double> yaws;
  std::vector<double> gains;
  double min_yaw_step = 2 * M_PI / min_yaw_samples;
  double yaw_step = 2 * M_PI / yaw_samples_;
  int aditional_angles = (360 - min_yaw_samples) / min_yaw_samples;

  for (int k = 0; k < min_yaw_samples; ++k) {
    double yaw_optimized = k * min_yaw_step / M_PI * 180.0f;
    double gain_optimized = 0;
    double max_gain_optimized = 0;
    for (int fov = -fov_y / 2; fov < fov_y / 2; fov++) {
      int theta = yaw_optimized + fov;
      if (theta < -180)
        theta += 360;
      if (theta > 180)
        theta -= 360;
      gain_optimized += gain_per_yaw[theta];
      max_gain_optimized += max_gain_per_yaw[theta];
    }

    yaws.push_back(yaw_optimized);
    gains.push_back(gain_optimized);

    if (gain_optimized > best_gain) {
      best_gain = gain_optimized;
      best_max_gain = max_gain_optimized;
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
      double max_gain_optimized = 0;
      for (int fov = -fov_y / 2; fov < fov_y / 2; fov++) {
        int theta = yaw_optimized + fov;
        if (theta < -180)
          theta += 360;
        if (theta > 180)
          theta -= 360;
        gain_optimized += gain_per_yaw[theta];
        max_gain_optimized += max_gain_per_yaw[theta];
      }

      if (gain_optimized > best_gain) {
        best_gain = gain_optimized;
        best_max_gain = max_gain_optimized;
        best_yaw = yaw_optimized;
      }
    }
  }

  gain = best_gain;
  max_gain = best_max_gain;
  yaw = M_PI * best_yaw / 180.f;

  //auto end = std::chrono::high_resolution_clock::now();
  //std::chrono::duration<double> elapsed = end - start;
  //std::chrono::duration<double> intermediate_elapsed = intermediate - start;
  //std::cout << "First Part AEPGain took " << intermediate_elapsed.count() << " seconds." << std::endl;
  //std::cout << "AEPGain took " << elapsed.count() << " seconds." << std::endl;

  return std::make_tuple(gain, max_gain, yaw);
}

double GainEvaluator::computeGainOptimizedwithMultiplePriorsFixed(const std::vector<eth_mav_msgs::EigenTrajectoryPoint>& previous_poses, const eth_mav_msgs::EigenTrajectoryPoint& pose) {
  CHECK_NOTNULL(tsdf_layer_);

  //auto start = std::chrono::high_resolution_clock::now();
  frustums.clear();
  frustums.reserve(previous_poses.size());
  for (int i = 0; i < previous_poses.size(); ++i) {
    // Set the camera pose to the previous one to get the previous Frustum
    prev_cam_model_.setBodyPose(voxblox::Transformation(
        previous_poses[i].orientation_W_B.cast<float>(), previous_poses[i].position_W.cast<float>()));

    frustums.push_back(prev_cam_model_.getBoundingPlanes());
  }
  
  /*// Set the camera pose to the previous one to get the previous Frustum
  prev_cam_model_.setBodyPose(voxblox::Transformation(
      previous_pose.orientation_W_B.cast<float>(), previous_pose.position_W.cast<float>()));

  bounding_planes_ = prev_cam_model_.getBoundingPlanes();*/

  // Now set camera model to the current pose
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

  voxblox::Point vec_voxblox;
  Eigen::Vector3d vec;
  
  double min_x = static_cast<double>(min_x_);
  double min_y = static_cast<double>(min_y_);
  double min_z = static_cast<double>(min_z_);
  double max_x = static_cast<double>(max_x_);
  double max_y = static_cast<double>(max_y_);
  double max_z = static_cast<double>(max_z_);

  double start_dist, intersect_len;

  int id = 0;
  for (theta = yaw - fov_y/2; theta < yaw + fov_y/2; theta += dtheta) {
    theta_rad = M_PI * theta / 180.0f;
    for (phi = 90 - fov_p / 2 + camera_pitch_; phi < 90 + fov_p / 2 + camera_pitch_; phi += dphi) {
      phi_rad = M_PI * phi / 180.0f;
      double g = 0.0;

      voxblox::Point ray_dir(cos(theta_rad) * sin(phi_rad), sin(theta_rad) * sin(phi_rad), cos(phi_rad));
      std::vector<std::pair<double, double>> skip_intervals = rayMultiFrustumIntersectionSegment(camera_center, ray_dir, dr_, r_max_);

      size_t skip_idx = 0;
      for (r = dr_; r < r_max_; r += dr_) {
        if (skip_idx < skip_intervals.size() && r >= skip_intervals[skip_idx].first) {
          r = skip_intervals[skip_idx].second;
          ++skip_idx;
          if (r >= r_max_) {
            break;
          }
          continue;
        }

        vec[0] = camera_center.x() + r * cos(theta_rad) * sin(phi_rad);
        vec[1] = camera_center.y() + r * sin(theta_rad) * sin(phi_rad);
        vec[2] = camera_center.z() + r * cos(phi_rad);
        
        vec_voxblox = vec.cast<float>();

        /*if (isPointInView(vec_voxblox, first_node)) {
          continue;
        }*/

        /*if (isInsideAABB(vec, aabb_min, aabb_max, first_node)) {
          break;
        }*/

        if (vec[0] < min_x || vec[0] > max_x || 
        vec[1] < min_y || vec[1] > max_y || 
        vec[2] < min_z || vec[2] > max_z) {
          continue;
        }

        VoxelStatus node = getVoxelStatus(vec);

        if (node == kOccupied) {
          break;
        } else if (node == kFree) {
          continue;
        } else if (node == kUnknown) {
          g += (2 * r * r * dr_ + 1 / 6 * dr_ * dr_ * dr_) * dtheta_rad * sin(phi_rad) * sin(dphi_rad / 2);
          /*if (isFrontierVoxel(vec)) {
            g += (2 * r * r * dr_ + 1 / 6 * dr_ * dr_ * dr_) * dtheta_rad * sin(phi_rad) * sin(dphi_rad / 2) / (voxel_size_ * voxel_size_ * voxel_size_);
          }*/
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
    for (phi = 90 - fov_p / 2 + camera_pitch_; phi < 90 + fov_p / 2 + camera_pitch_; phi += dphi) {
      phi_rad = M_PI * phi / 180.0f;

      bool occupied_ray = false;
      double g = 0;
      voxblox::Pointcloud voxels_ray;
      for (r = 0; r < r_max_; r += dr_) {
        vec[0] = camera_center.x() + r * cos(theta_rad) * sin(phi_rad);
        vec[1] = camera_center.y() + r * sin(theta_rad) * sin(phi_rad);
        vec[2] = camera_center.z() + r * cos(phi_rad);

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
          /*if (isFrontierVoxel(vec)) {
            g += (2 * r * r * dr_ + 1 / 6 * dr_ * dr_ * dr_) * dtheta_rad * sin(phi_rad) * sin(dphi_rad / 2);
          } else {
            continue;
          }*/
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

void GainEvaluator::computeCost(std::shared_ptr<rrt_star::Node>& new_node) {
    new_node->cost = new_node->parent->cost + (new_node->point.head(3) - new_node->parent->point.head(3)).norm();
}

void GainEvaluator::computeScore(std::shared_ptr<rrt_star::Node>& new_node, double lambda) {
    new_node->score = new_node->parent->score + new_node->gain * exp(-lambda * new_node->cost);
}

void GainEvaluator::computeCost(std::shared_ptr<geo_rrt::Node>& new_node) {
    new_node->cost = new_node->parent->cost + (new_node->pose.head(3) - new_node->parent->pose.head(3)).norm();
}

void GainEvaluator::computeScore(std::shared_ptr<geo_rrt::Node>& new_node, double lambda) {
    new_node->score = new_node->parent->score + new_node->path_gain * exp(-lambda * new_node->cost);
}

void GainEvaluator::computeCostTwo(std::shared_ptr<kino_rrt_star::Trajectory>& new_trajectory) {
    new_trajectory->cost1 = new_trajectory->parent->cost1 + new_trajectory->cost1;
    new_trajectory->cost2 = new_trajectory->parent->cost2 + new_trajectory->cost2;
}

void GainEvaluator::computeScore(std::shared_ptr<kino_rrt_star::Trajectory>& new_trajectory, double lambda1, double lambda2) {
    new_trajectory->score = new_trajectory->parent->score + new_trajectory->gain * exp(-lambda1 * new_trajectory->cost1 - lambda2 * new_trajectory->cost2);
}

void GainEvaluator::computeSingleScore(std::shared_ptr<kino_rrt_star::Trajectory>& new_trajectory, double lambda1, double lambda2) {
    new_trajectory->score = new_trajectory->gain * exp(-lambda1 * new_trajectory->cost1 - lambda2 * new_trajectory->cost2);
}

void GainEvaluator::computeCostTwo(std::shared_ptr<improved_krrt::Node>& new_node) {
    new_node->cost1 = new_node->parent->cost1 + new_node->cost1;
    new_node->cost2 = new_node->parent->cost2 + new_node->cost2;
}

void GainEvaluator::computeScore(std::shared_ptr<improved_krrt::Node>& new_node, double lambda1, double lambda2) {
    //new_node->score = new_node->parent->score + new_node->gain * exp(-lambda1 * new_node->cost1 - lambda2 * new_node->cost2);
    // If the time cost is smaller than 1, then the planner will have a score bigger than the gain (should not occur)
    if ((lambda1 * new_node->cost1 + lambda2 * new_node->cost2) < 1) {
      new_node->score = new_node->gain;
    } else {
      new_node->score = new_node->gain / (lambda1 * new_node->cost1 + lambda2 * new_node->cost2);
    }
}

void GainEvaluator::computeSingleScore(std::shared_ptr<improved_krrt::Node>& new_node, double lambda1, double lambda2) {
    //new_node->score = new_node->gain * exp(-lambda1 * new_node->cost1 - lambda2 * new_node->cost2);
    // If the time cost is smaller than 1, then the planner will have a score bigger than the gain (should not occur)
    if ((lambda1 * new_node->cost1 + lambda2 * new_node->cost2) < 1) {
      new_node->score = new_node->gain;
    } else {
      new_node->score = new_node->gain / (lambda1 * new_node->cost1 + lambda2 * new_node->cost2);
    }
}

voxblox::CameraModel& GainEvaluator::getCameraModel() { return cam_model_; }

const voxblox::CameraModel& GainEvaluator::getCameraModel() const {
  return cam_model_;
}
