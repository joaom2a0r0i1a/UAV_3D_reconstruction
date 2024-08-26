#include <ros/ros.h>
#include <eth_trajectory_generation/timing.h>
#include <voxblox/integrator/integrator_utils.h>

#include "motion_planning_python/gain_evaluator.h"

//namespace mav_planning {

GainEvaluator::GainEvaluator() {
  // Fire Station
  /*min_x_ = -11;
  max_x_ = 11;
  min_y_ = -6.5;
  max_y_ = 6.5;
  min_z_ = 0.0;
  max_z_ = 11.5;*/

  /*// School
  min_x_ = -17;
  max_x_ = 17;
  min_y_ = -12;
  max_y_ = 7.0;
  min_z_ = 0.0;
  max_z_ = 14.5;*/

  min_x_ = -17;
  max_x_ = 17;
  min_y_ = -12;
  max_y_ = 7.0;
  min_z_ = 0.0;
  max_z_ = 14.5;

  /*// Grocery Store
  min_x_ = -16.5;
  max_x_ = 16.5;
  min_y_ = -14.5;
  max_y_ = 9.0;
  min_z_ = 0.0;
  max_z_ = 8.5;*/

  // Police Station
  /*min_x_ = -7;
  max_x_ = 7;
  min_y_ = -8.5;
  max_y_ = 8.5;
  min_z_ = 0.0;
  max_z_ = 9.0;*/

  /*min_x_ = -7;
  max_x_ = 7;
  min_y_ = -9.5;
  max_y_ = 9.5;
  min_z_ = 0.0;
  max_z_ = 9.0;*/

  // House
  /*min_x_ = -5;
  max_x_ = 7.5;
  min_y_ = -4.5;
  max_y_ = 5.5;
  min_z_ = 0.0;
  max_z_ = 7.5;*/

  /*min_x_ = -6;
  max_x_ = 8.5;
  min_y_ = -5.5;
  max_y_ = 6.0;
  min_z_ = 0.0;
  max_z_ = 7.5;*/

  fov_y_rad_ = 1.51844;
  fov_p_rad_ = 1.01229;
  r_max_ = 5.0;
  yaw_samples = 15;

  /*min_x_ = -11;
  max_x_ = 11;
  min_y_ = -6.0;
  max_y_ = 6.0;
  min_z_ = 0.0;
  max_z_ = 11.5;*/

  /*min_x_ = -14;
  max_x_ = 14;
  min_y_ = -10;
  max_y_ = 10;
  min_z_ = 0;
  max_z_ = 14;*/
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
    //std::cout << "Voxel weight: " << voxel->weight << std::endl;
    //std::cout << "Voxel distance: " << voxel->distance << std::endl;
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

double GainEvaluator::evaluateExplorationGainWithRaycasting(
    const eth_mav_msgs::EigenTrajectoryPoint& pose, int modulus) {
  CHECK_NOTNULL(tsdf_layer_);

  //auto start = std::chrono::high_resolution_clock::now();

  //eth_trajectory_generation::timing::Timer timer_gain(
  //    "exploration/exp_gain_raycast");

  cam_model_.setBodyPose(voxblox::Transformation(
      pose.orientation_W_B.cast<float>(), pose.position_W.cast<float>()));

  // Get the center of the camera to raycast to.
  voxblox::Transformation camera_pose = cam_model_.getCameraPose();
  voxblox::Point camera_center = camera_pose.getPosition();

  double num_unknown = 0.0;
  double num_occluded = 0.0;
  double num_free = 0.0;
  double num_occupied = 0.0;

  // This is a set of all the checked voxels so that they don't get checked
  // multiple times...
  voxblox::HierarchicalIndexSet checked_voxels_set;

  // Get the plane bounds for the back plane of the frustum and just iterate
  // over these points.
  // For each voxel, cast a ray to the camera center and check whether it is
  // occluded or not.
  double gain = 0.0;
  int checked_voxels = 0;
  int voxel_index = 0;

  // Get the three points defining the back plane of the camera frustum.
  voxblox::AlignedVector<voxblox::Point> plane_points;
  cam_model_.getFarPlanePoints(&plane_points);

  // We map the plane into u and v coordinates, which are the plane's coordinate
  // system, with the origin at plane_points[1] and outer bounds at
  // plane_points[0] and plane_points[2].
  Eigen::Vector3f u_distance = plane_points[0] - plane_points[1];
  Eigen::Vector3f u_slope = u_distance.normalized();
  int u_max = static_cast<int>(
      std::ceil(u_distance.norm() * voxel_size_inv_));  // Round this up.

  Eigen::Vector3f v_distance = plane_points[2] - plane_points[1];
  Eigen::Vector3f v_slope = v_distance.normalized();
  int v_max = static_cast<int>(
      std::ceil(v_distance.norm() * voxel_size_inv_));  // Round this up.

  /*min_x_ = -11;
  max_x_ = 11;
  min_y_ = -6.5;
  max_y_ = 6.5;
  min_z_ = 0;
  max_z_ = 11.5;*/

  // We then iterate over all the voxels in the coordinate space of the back
  // bounding plane of the frustum.
  Eigen::Vector3f pos = plane_points[1];
  for (int u = 0; u < u_max; u++) {
    for (int v = 0; v < v_max; v++) {
      if (voxel_index % modulus != 0) {
        voxel_index++;
        continue;
      }
      voxel_index++;

      // Get the 'real' coordinates back from the plane coordinate space.
      pos = plane_points[1] + u * u_slope * voxel_size_ +
            v * v_slope * voxel_size_;

      // Ensure that the ray intersects the bounding box before processing
      //if (!isRayIntersectingBoundingBox(camera_center, pos)) {
      //  continue;
      //}

      // Get the block + voxel index of this voxel by projecting it into
      // the voxel grid and then computing from the global index.
      // This is a truncating cast, which is I think what we want in this
      // case.
      voxblox::GlobalIndex global_voxel_idx =
          (voxel_size_inv_ * pos).cast<voxblox::LongIndexElement>();
      voxblox::BlockIndex block_index =
          voxblox::getBlockIndexFromGlobalVoxelIndex(global_voxel_idx,
                                                     voxels_per_side_inv_);
      voxblox::VoxelIndex voxel_index = voxblox::getLocalFromGlobalVoxelIndex(
          global_voxel_idx, voxels_per_side_);

      // Should we check if we already cast this?
      if (checked_voxels_set[block_index].count(voxel_index) > 0) {
        continue;
        std::cout << "THIS MIGHT NOT BE NEEDED" << std::endl;
      }
      // Otherwise we should probably cast the ray through it and then
      // look up all the voxels and count them.
      const voxblox::Point start_scaled = camera_center * voxel_size_inv_;
      const voxblox::Point end_scaled = pos * voxel_size_inv_;

      voxblox::AlignedVector<voxblox::GlobalIndex> global_voxel_indices;
      voxblox::castRay(start_scaled, end_scaled, &global_voxel_indices);

      // Iterate over all the voxels in the index in order.
      // Put them in the checked queue, and classify them. We're starting from
      // the camera center to the current pose, so this defines how we handle
      // occlusions.
      int unknown_ray = 0;
      int occluded_ray = 0;
      int free_ray = 0;
      int occupied_ray = 0;
      bool ray_occluded = false;
      for (int i = 0; i < global_voxel_indices.size(); i++) {
        const voxblox::GlobalIndex& global_voxel_idx = global_voxel_indices[i];
        voxblox::BlockIndex block_index_ray =
            voxblox::getBlockIndexFromGlobalVoxelIndex(global_voxel_idx,
                                                       voxels_per_side_inv_);
        voxblox::VoxelIndex voxel_index_ray =
            voxblox::getLocalFromGlobalVoxelIndex(global_voxel_idx,
                                                  voxels_per_side_);

        bool voxel_checked = (checked_voxels_set[block_index_ray].count(voxel_index_ray) > 0);
        if (voxel_checked) {
          continue;
        } else {
          voxblox::Point recovered_pos = global_voxel_idx.cast<float>() * voxel_size_;
          if (!cam_model_.isPointInView(recovered_pos) ||
              recovered_pos.x() < min_x_ || recovered_pos.x() > max_x_ ||
              recovered_pos.y() < min_y_ || recovered_pos.y() > max_y_ ||
              recovered_pos.z() < min_z_ || recovered_pos.z() > max_z_) {
            continue;
          }
        }

        // This is as far as we need to go if this ray is already occluded.
        if (ray_occluded) {
          occluded_ray++;
          checked_voxels++;
          checked_voxels_set[block_index_ray].insert(voxel_index_ray);
          continue;
        }

        // Otherwise look up this voxel and add it to checked.
        const voxblox::Block<voxblox::TsdfVoxel>::Ptr block_ptr =
            tsdf_layer_->getBlockPtrByIndex(block_index_ray);
        if (block_ptr) {
          // If this block exists, get the voxel.
          const voxblox::TsdfVoxel& voxel =
              block_ptr->getVoxelByVoxelIndex(voxel_index_ray);
          if (voxel.weight <= 1e-2) {
            unknown_ray++;
          } else if (voxel.distance <= 0.0) {
            // This is an occupied voxel! Mark all the stuff behind
            // it as occluded.
            occupied_ray++;
            ray_occluded = true;
          } else {
            free_ray++;
          }
        } else {
          unknown_ray++;
        }

        checked_voxels++;
        checked_voxels_set[block_index_ray].insert(voxel_index_ray);
      }

      // Now that we have the whole ray, add up the counts.
      num_unknown += unknown_ray;
      num_free += free_ray;
      num_occluded += occluded_ray;
      num_occupied += occupied_ray;
    }
  }
  // Divide percentages by the checked voxels.
  //num_unknown /= checked_voxels;

  //timer_gain.Stop();
  //auto end = std::chrono::high_resolution_clock::now();
  //std::chrono::duration<double> elapsed = end - start;
  //std::cout << "evaluateExplorationGainWithRaycasting took " << elapsed.count() << " seconds." << std::endl;
  return num_unknown;
}

bool GainEvaluator::isRayIntersectingBoundingBox(const voxblox::Point& start, const voxblox::Point& end) {
  // Calculate the intersection of the ray with the bounding box
  float tmin = (min_x_ - start.x()) / (end.x() - start.x());
  float tmax = (max_x_ - start.x()) / (end.x() - start.x());
  if (tmin > tmax) std::swap(tmin, tmax);

  float tymin = (min_y_ - start.y()) / (end.y() - start.y());
  float tymax = (max_y_ - start.y()) / (end.y() - start.y());
  if (tymin > tymax) std::swap(tymin, tymax);

  if ((tmin > tymax) || (tymin > tmax))
    return false;

  if (tymin > tmin)
    tmin = tymin;
  if (tymax < tmax)
    tmax = tymax;

  float tzmin = (min_z_ - start.z()) / (end.z() - start.z());
  float tzmax = (max_z_ - start.z()) / (end.z() - start.z());
  if (tzmin > tzmax) std::swap(tzmin, tzmax);

  if ((tmin > tzmax) || (tzmin > tmax))
    return false;

  if (tzmin > tmin)
    tmin = tzmin;
  if (tzmax < tmax)
    tmax = tzmax;

  return true;
}

double GainEvaluator::computeGainRaycasting(const eth_mav_msgs::EigenTrajectoryPoint& pose, int modulus) {
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

  double dr = 0.2;
  double dphi_rad = dr / r_max_;
  double dtheta_rad = dr / r_max_;
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
      for (r = 0; r < r_max_; r += dr) {
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
          g += (2 * r * r * dr + 1 / 6 * dr * dr * dr) * dtheta_rad * sin(phi_rad) * sin(dphi_rad / 2);
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

std::pair<double, double> GainEvaluator::computeGainRaycastingFromSampledYaw(eth_mav_msgs::EigenTrajectoryPoint& position) {
  double best_gain;
  double best_yaw;

  //auto start = std::chrono::high_resolution_clock::now();

  for (int k = 0; k < yaw_samples; ++k) {
    double yaw = k * 2 * M_PI / yaw_samples;
    //position.position_W = node->point.head(3);
    position.setFromYaw(yaw);
    double gain = computeGainRaycasting(position);
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
  double best_gain;
  double best_yaw;

  int min_yaw_samples = ceil(2 * M_PI / fov_y_rad_);

  std::vector<double> yaws;
  std::vector<double> gains;
  double min_yaw_step = 2 * M_PI / min_yaw_samples;
  double yaw_step = 2 * M_PI / yaw_samples;
  int aditional_angles = (yaw_samples - min_yaw_samples) / min_yaw_samples;

  //auto start = std::chrono::high_resolution_clock::now();

  for (int k = 0; k < min_yaw_samples; ++k) {
    double yaw = k * min_yaw_step;
    position.setFromYaw(yaw);
    double gain = computeGainRaycasting(position);

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
    int prev = (i - 1 + min_yaw_samples) % min_yaw_samples;
    int next = (i + 1) % min_yaw_samples;

    if ((gains[i] + gains[next] > best_gain)) {
      filteredYaws.push_back(yaws[i]);
    }

    /*if ((gains[i] + gains[next] > best_gain) || (gains[i] + gains[prev] > best_gain)) {
      filteredYaws.push_back(yaws[i]);
    }*/
  }

  for (int j = 0; j < filteredYaws.size(); ++j) {
    for (int l = 0; l < aditional_angles; ++l) {
      double yaw = filteredYaws[j] + yaw_step * l;
      position.setFromYaw(yaw);
      double gain = computeGainRaycasting(position);

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

  double dr = 0.2, dphi = 10, dtheta = 10;
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

      double g = 0;
      bool occupied_ray = false;
      for (r = 0; r < r_max_; r += dr) {
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
          g += (2 * r * r * dr + 1 / 6 * dr * dr * dr) * dtheta_rad * sin(phi_rad) * sin(dphi_rad / 2);
        }

        /*double distance = 0.0;
        if (esdf_map_->getDistanceAtPosition(vec, &distance)) {
          if (distance < voxel_size_) {
            break;
          }
        } else {
          g += (2 * r * r * dr + 1 / 6 * dr * dr * dr) * dtheta_rad * sin(phi_rad) * sin(dphi_rad / 2);
          std::cout << "Position X: " << vec[0] << std::endl;
          std::cout << "Position Y: " << vec[1] << std::endl;
          std::cout << "Position Z: " << vec[2] << std::endl;
        }*/

        /*std::cout << "Position X: " << vec[0] << std::endl;
        std::cout << "Position Y: " << vec[1] << std::endl;
        std::cout << "Position Z: " << vec[2] << std::endl;*/

        /*const voxblox::Point start_scaled = camera_center * voxel_size_inv_;
        const voxblox::Point end_scaled = vec * voxel_size_inv_;
        
        voxblox::AlignedVector<voxblox::GlobalIndex> global_voxel_indices;
        voxblox::castRay(start_scaled, end_scaled, &global_voxel_indices);

        int unknown_ray = 0;
        int free_ray = 0;
        bool ray_occluded = false;
        const voxblox::GlobalIndex& global_voxel_idx = global_voxel_indices.back();
        voxblox::BlockIndex block_index_ray =
            voxblox::getBlockIndexFromGlobalVoxelIndex(global_voxel_idx,
                                                      voxels_per_side_inv_);
        voxblox::VoxelIndex voxel_index_ray =
            voxblox::getLocalFromGlobalVoxelIndex(global_voxel_idx,
                                                  voxels_per_side_);

        // Otherwise look up this voxel and add it to checked.
        const voxblox::Block<voxblox::TsdfVoxel>::Ptr block_ptr =
            tsdf_layer_->getBlockPtrByIndex(block_index_ray);
        if (block_ptr) {
          // If this block exists, get the voxel.
          const voxblox::TsdfVoxel& voxel =
              block_ptr->getVoxelByVoxelIndex(voxel_index_ray);
          if (voxel.weight < 1e-6) {
            g += (2 * r * r * dr + 1 / 6 * dr * dr * dr) * dtheta_rad * sin(phi_rad) * sin(dphi_rad / 2);
          } else if (voxel.distance <= 0.0) {
          //if (voxel.distance <= 0.0 && voxel.weight >= 1e-6) {
            ray_occluded = true;
            break;
          } 
        } else {
          g += (2 * r * r * dr + 1 / 6 * dr * dr * dr) * dtheta_rad * sin(phi_rad) * sin(dphi_rad / 2);
        }*/
      }
      
      /*if (!occupied_ray) {
        gain += g;
      }*/

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

  double dr = 0.2, dphi = 10, dtheta = 10;
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
      for (r = 0; r < r_max_; r += dr) {
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
          g += (2 * r * r * dr + 1 / 6 * dr * dr * dr) * dtheta_rad * sin(phi_rad) * sin(dphi_rad / 2);
        }

        /*double distance = 0.0;
        if (esdf_map_->getDistanceAtPosition(vec, &distance)) {
          if (distance < voxel_size_) {
            break;
          }
        } else {
          g += (2 * r * r * dr + 1 / 6 * dr * dr * dr) * dtheta_rad * sin(phi_rad) * sin(dphi_rad / 2);
          std::cout << "Position X: " << vec[0] << std::endl;
          std::cout << "Position Y: " << vec[1] << std::endl;
          std::cout << "Position Z: " << vec[2] << std::endl;
        }*/

        /*std::cout << "Position X: " << vec[0] << std::endl;
        std::cout << "Position Y: " << vec[1] << std::endl;
        std::cout << "Position Z: " << vec[2] << std::endl;*/

        /*const voxblox::Point start_scaled = camera_center * voxel_size_inv_;
        const voxblox::Point end_scaled = vec * voxel_size_inv_;
        
        voxblox::AlignedVector<voxblox::GlobalIndex> global_voxel_indices;
        voxblox::castRay(start_scaled, end_scaled, &global_voxel_indices);

        int unknown_ray = 0;
        int free_ray = 0;
        bool ray_occluded = false;
        const voxblox::GlobalIndex& global_voxel_idx = global_voxel_indices.back();
        voxblox::BlockIndex block_index_ray =
            voxblox::getBlockIndexFromGlobalVoxelIndex(global_voxel_idx,
                                                      voxels_per_side_inv_);
        voxblox::VoxelIndex voxel_index_ray =
            voxblox::getLocalFromGlobalVoxelIndex(global_voxel_idx,
                                                  voxels_per_side_);

        // Otherwise look up this voxel and add it to checked.
        const voxblox::Block<voxblox::TsdfVoxel>::Ptr block_ptr =
            tsdf_layer_->getBlockPtrByIndex(block_index_ray);
        if (block_ptr) {
          // If this block exists, get the voxel.
          const voxblox::TsdfVoxel& voxel =
              block_ptr->getVoxelByVoxelIndex(voxel_index_ray);
          if (voxel.weight < 1e-6) {
            g += (2 * r * r * dr + 1 / 6 * dr * dr * dr) * dtheta_rad * sin(phi_rad) * sin(dphi_rad / 2);
          } else if (voxel.distance <= 0.0) {
          //if (voxel.distance <= 0.0 && voxel.weight >= 1e-6) {
            ray_occluded = true;
            break;
          } 
        } else {
          g += (2 * r * r * dr + 1 / 6 * dr * dr * dr) * dtheta_rad * sin(phi_rad) * sin(dphi_rad / 2);
        }*/
      }
      
      /*if (!occupied_ray) {
        gain += g;
        gain_per_yaw[theta] += g;
      } else {
        gain += 0.0;
        gain_per_yaw[theta] += 0.0;
      }*/

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

  double dr = 0.2, dphi = 10, dtheta = 10;
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
      for (r = 0; r < r_max_; r += dr) {
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
          g += (2 * r * r * dr + 1 / 6 * dr * dr * dr) * dtheta_rad * sin(phi_rad) * sin(dphi_rad / 2);
          Eigen::Vector3d Voxel;
          getVoxelCenter(&Voxel, vec);
          
          voxblox::Point VoxelCenter;
          VoxelCenter[0] = Voxel[0];
          VoxelCenter[1] = Voxel[1];
          VoxelCenter[2] = Voxel[2];
          //voxels.push_back(VoxelCenter);
          voxels_ray.push_back(VoxelCenter);
        }

        /*double distance = 0.0;
        if (esdf_map_->getDistanceAtPosition(vec, &distance)) {
          if (distance < voxel_size_) {
            break;
          }
        } else {
          g += (2 * r * r * dr + 1 / 6 * dr * dr * dr) * dtheta_rad * sin(phi_rad) * sin(dphi_rad / 2);
          std::cout << "Position X: " << vec[0] << std::endl;
          std::cout << "Position Y: " << vec[1] << std::endl;
          std::cout << "Position Z: " << vec[2] << std::endl;
        }*/

        /*std::cout << "Position X: " << vec[0] << std::endl;
        std::cout << "Position Y: " << vec[1] << std::endl;
        std::cout << "Position Z: " << vec[2] << std::endl;*/

        /*const voxblox::Point start_scaled = camera_center * voxel_size_inv_;
        const voxblox::Point end_scaled = vec * voxel_size_inv_;
        
        voxblox::AlignedVector<voxblox::GlobalIndex> global_voxel_indices;
        voxblox::castRay(start_scaled, end_scaled, &global_voxel_indices);

        int unknown_ray = 0;
        int free_ray = 0;
        bool ray_occluded = false;
        const voxblox::GlobalIndex& global_voxel_idx = global_voxel_indices.back();
        voxblox::BlockIndex block_index_ray =
            voxblox::getBlockIndexFromGlobalVoxelIndex(global_voxel_idx,
                                                      voxels_per_side_inv_);
        voxblox::VoxelIndex voxel_index_ray =
            voxblox::getLocalFromGlobalVoxelIndex(global_voxel_idx,
                                                  voxels_per_side_);

        // Otherwise look up this voxel and add it to checked.
        const voxblox::Block<voxblox::TsdfVoxel>::Ptr block_ptr =
            tsdf_layer_->getBlockPtrByIndex(block_index_ray);
        if (block_ptr) {
          // If this block exists, get the voxel.
          const voxblox::TsdfVoxel& voxel =
              block_ptr->getVoxelByVoxelIndex(voxel_index_ray);
          if (voxel.weight < 1e-6) {
            g += (2 * r * r * dr + 1 / 6 * dr * dr * dr) * dtheta_rad * sin(phi_rad) * sin(dphi_rad / 2);
            voxels.push_back(vec);
          } else if (voxel.distance <= 0.0) {
          //if (voxel.distance <= 0.0 && voxel.weight >= 1e-6) {
            ray_occluded = true;
            break;
          } 
        } else {
          g += (2 * r * r * dr + 1 / 6 * dr * dr * dr) * dtheta_rad * sin(phi_rad) * sin(dphi_rad / 2);
          voxels.push_back(vec);
        }*/
      }

      /*if (!occupied_ray) {
        voxels.insert(voxels.end(), voxels_ray.begin(), voxels_ray.end());
      }*/

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

  /*min_x_ = -11;
  max_x_ = 11;
  min_y_ = -6.5;
  max_y_ = 6.5;
  min_z_ = 0;
  max_z_ = 11.5;*/

  /*min_x_ = -14;
  max_x_ = 14;
  min_y_ = -10;
  max_y_ = 10;
  min_z_ = 0;
  max_z_ = 14;*/

  const float min_x = std::max(aabb_min.x(), min_x_);
  const float max_x = std::min(aabb_max.x(), max_x_);
  const float min_y = std::max(aabb_min.y(), min_y_);
  const float max_y = std::min(aabb_max.y(), max_y_);
  const float min_z = std::max(aabb_min.z(), min_z_);
  const float max_z = std::min(aabb_max.z(), max_z_);

  //return std::max(aabb_min.z(), min_z_);

  /*for (pos[0] = std::max(camera_center[0] - gain_range_, min_x_);
      pos[0] < std::min(camera_center[0] + gain_range_, max_x_); pos[0] += voxel_size_) {
    for (pos[1] = std::max(camera_center[1] - gain_range_, min_y_);
        pos[1] < std::min(camera_center[1] + gain_range_, max_y_); pos[1] += voxel_size_) {
      for (pos[2] = std::max(camera_center[2] - gain_range_, min_z_);
          pos[2] < std::min(camera_center[2] + gain_range_, max_z_); pos[2] += voxel_size_) {*/

  /*for (pos.x() = std::max(aabb_min.x(), min_x_); pos.x() < std::min(aabb_max.x(), max_x_); pos.x() += voxel_size_) {
    for (pos.y() = std::max(aabb_min.y(), min_y_); pos.y() < std::min(aabb_max.y(), max_y_); pos.y() += voxel_size_) {
      for (pos.z() = std::max(aabb_min.z(), min_z_); pos.z() < std::min(aabb_max.z(), max_z_); pos.z() += voxel_size_) {*/

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
          // If it's not occluded, ACTUALLY look up this voxel.
          /*const voxblox::Block<voxblox::TsdfVoxel>::Ptr block_ptr = tsdf_layer_->getBlockPtrByIndex(block_index);
          if (block_ptr) {
            const voxblox::TsdfVoxel& voxel = block_ptr->getVoxelByCoordinates(pos);
            if (voxel.weight <= 1e-6) {
              num_unknown++;
            } else if (voxel.distance >= 0.0) {
            //if (voxel.distance >= 0.0) {
              num_free++;
            } else {
              num_occupied++;
            }
          } else {
            num_unknown++;
          }*/
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

  // Divide percentages by the checked voxels.
  //num_unknown /= checked_voxels;
  //auto end = std::chrono::high_resolution_clock::now();
  //std::chrono::duration<double> elapsed = end - start;
  //std::cout << "BircherGain took " << elapsed.count() << " seconds." << std::endl;

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

voxblox::CameraModel& GainEvaluator::getCameraModel() { return cam_model_; }

const voxblox::CameraModel& GainEvaluator::getCameraModel() const {
  return cam_model_;
}
