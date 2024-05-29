#include <eth_trajectory_generation/timing.h>
#include <voxblox/integrator/integrator_utils.h>

#include "gain_evaluator.h"

//namespace mav_planning {

GainEvaluator::GainEvaluator() {
  min_x_ = -11;
  max_x_ = 11;
  min_y_ = -6.5;
  max_y_ = 6.5;
  min_z_ = 0;
  max_z_ = 11.5;
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

void GainEvaluator::setBounds(float& min_x, float& min_y, float& min_z,
                              float& max_x, float& max_y, float& max_z, float& gain_range) {
  min_x_ = min_x;
  min_y_ = min_y;
  min_z_ = min_z;
  max_x_ = max_x;
  max_y_ = max_y;
  max_z_ = max_z;
  gain_range_ = gain_range;
  
}

void GainEvaluator::initialize_neighbour_voxels(float voxel_size) {
  auto vs = voxel_size;
  if (!p_accurate_frontiers_) {
    neighbor_voxels_[0] = Eigen::Vector3d(vs, 0, 0);
    neighbor_voxels_[1] = Eigen::Vector3d(-vs, 0, 0);
    neighbor_voxels_[2] = Eigen::Vector3d(0, vs, 0);
    neighbor_voxels_[3] = Eigen::Vector3d(0, -vs, 0);
    neighbor_voxels_[4] = Eigen::Vector3d(0, 0, vs);
    neighbor_voxels_[5] = Eigen::Vector3d(0, 0, -vs);
  } else {
    neighbor_voxels_[0] = Eigen::Vector3d(vs, 0, 0);
    neighbor_voxels_[1] = Eigen::Vector3d(vs, vs, 0);
    neighbor_voxels_[2] = Eigen::Vector3d(vs, -vs, 0);
    neighbor_voxels_[3] = Eigen::Vector3d(vs, 0, vs);
    neighbor_voxels_[4] = Eigen::Vector3d(vs, vs, vs);
    neighbor_voxels_[5] = Eigen::Vector3d(vs, -vs, vs);
    neighbor_voxels_[6] = Eigen::Vector3d(vs, 0, -vs);
    neighbor_voxels_[7] = Eigen::Vector3d(vs, vs, -vs);
    neighbor_voxels_[8] = Eigen::Vector3d(vs, -vs, -vs);
    neighbor_voxels_[9] = Eigen::Vector3d(0, vs, 0);
    neighbor_voxels_[10] = Eigen::Vector3d(0, -vs, 0);
    neighbor_voxels_[11] = Eigen::Vector3d(0, 0, vs);
    neighbor_voxels_[12] = Eigen::Vector3d(0, vs, vs);
    neighbor_voxels_[13] = Eigen::Vector3d(0, -vs, vs);
    neighbor_voxels_[14] = Eigen::Vector3d(0, 0, -vs);
    neighbor_voxels_[15] = Eigen::Vector3d(0, vs, -vs);
    neighbor_voxels_[16] = Eigen::Vector3d(0, -vs, -vs);
    neighbor_voxels_[17] = Eigen::Vector3d(-vs, 0, 0);
    neighbor_voxels_[18] = Eigen::Vector3d(-vs, vs, 0);
    neighbor_voxels_[19] = Eigen::Vector3d(-vs, -vs, 0);
    neighbor_voxels_[20] = Eigen::Vector3d(-vs, 0, vs);
    neighbor_voxels_[21] = Eigen::Vector3d(-vs, vs, vs);
    neighbor_voxels_[22] = Eigen::Vector3d(-vs, -vs, vs);
    neighbor_voxels_[23] = Eigen::Vector3d(-vs, 0, -vs);
    neighbor_voxels_[24] = Eigen::Vector3d(-vs, vs, -vs);
    neighbor_voxels_[25] = Eigen::Vector3d(-vs, -vs, -vs);
  }
}

/*bool GainEvaluator::isFrontierVoxel(const Eigen::Vector3d& voxel) {
  // Check all neighboring voxels
  unsigned char voxel_state;
  if (!p_accurate_frontiers_) {
    for (int i = 0; i < 6; ++i) {
      voxel_state = map_->getVoxelState(voxel + neighbor_voxels_[i]);
      if (voxel_state == map::OccupancyMap::UNKNOWN) {
        continue;
      }
      if (p_surface_frontiers_) {
        return voxel_state == map::OccupancyMap::OCCUPIED;
      } else {
        return true;
      }
    }
  } else {
    for (int i = 0; i < 26; ++i) {
      voxel_state = map_->getVoxelState(voxel + neighbor_voxels_[i]);
      if (voxel_state == map::OccupancyMap::UNKNOWN) {
        continue;
      }
      if (p_surface_frontiers_) {
        return voxel_state == map::OccupancyMap::OCCUPIED;
      } else {
        return true;
      }
    }
  }
  return false;
}*/

double GainEvaluator::evaluateExplorationGainWithRaycasting(
    const eth_mav_msgs::EigenTrajectoryPoint& pose, int modulus) {
  CHECK_NOTNULL(tsdf_layer_);

  eth_trajectory_generation::timing::Timer timer_gain(
      "exploration/exp_gain_raycast");

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

  min_x_ = -11;
  max_x_ = 11;
  min_y_ = -6.5;
  max_y_ = 6.5;
  min_z_ = 0;
  max_z_ = 11.5;

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
      if (!isRayIntersectingBoundingBox(camera_center, pos)) {
        continue;
      }

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
        if (!voxel_checked) {
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
          if (!voxel_checked) {
            occluded_ray++;
            checked_voxels++;
            checked_voxels_set[block_index_ray].insert(voxel_index_ray);
          }
          continue;
        }

        // Otherwise look up this voxel and add it to checked.
        const voxblox::Block<voxblox::TsdfVoxel>::Ptr block_ptr =
            tsdf_layer_->getBlockPtrByIndex(block_index_ray);
        if (block_ptr) {
          // If this block exists, get the voxel.
          const voxblox::TsdfVoxel& voxel =
              block_ptr->getVoxelByVoxelIndex(voxel_index_ray);
          if (voxel.weight <= 1e-1) {
            if (!voxel_checked) {
              unknown_ray++;
            }
          } else if (voxel.distance <= 0.0) {
            // This is an occupied voxel! Mark all the stuff behind
            // it as occluded.
            if (!voxel_checked) {
              occupied_ray++;
            }
            ray_occluded = true;
          } else {
            if (!voxel_checked) {
              free_ray++;
            }
          }
        } else {
          if (!voxel_checked) {
            unknown_ray++;
          }
        }

        if (!voxel_checked) {
          checked_voxels++;
          checked_voxels_set[block_index_ray].insert(voxel_index_ray);
        }
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

  timer_gain.Stop();
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


double GainEvaluator::computeGain(const eth_mav_msgs::EigenTrajectoryPoint& pose, int modulus) {
  eth_trajectory_generation::timing::Timer timer_gain("gain_evaluator");

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

  min_x_ = -11;
  max_x_ = 11;
  min_y_ = -6.5;
  max_y_ = 6.5;
  min_z_ = 0;
  max_z_ = 11.5;

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
            if (voxel.weight > 1e-1 && voxel.distance <= 0.0) {
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
          const voxblox::Block<voxblox::TsdfVoxel>::Ptr block_ptr = tsdf_layer_->getBlockPtrByIndex(block_index);
          if (block_ptr) {
            const voxblox::TsdfVoxel& voxel = block_ptr->getVoxelByCoordinates(pos);
            if (voxel.weight <= 1e-2) {
              num_unknown++;
            } else if (voxel.distance >= 0.0) {
            //if (voxel.distance >= 0.0) {
              num_free++;
            } else {
              num_occupied++;
            }
          } else {
            num_unknown++;
          }
        }
      }
    }
  }
  timer_gain.Stop();

  // Divide percentages by the checked voxels.
  //num_unknown /= checked_voxels;

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

double GainEvaluator::computeGainInNeighbours(
    const eth_mav_msgs::EigenTrajectoryPoint& pose, int modulus) {
  CHECK_NOTNULL(tsdf_layer_);

  eth_trajectory_generation::timing::Timer timer_gain(
      "exploration/exp_gain_raycast");

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

        bool voxel_checked = false;
        // Check if this is already checked; we don't add it to the counts
        // in that case.
        if (checked_voxels_set[block_index_ray].count(voxel_index_ray) > 0) {
          voxel_checked = true;
        }
        voxblox::Point recovered_pos =
            global_voxel_idx.cast<float>() * voxel_size_;
        if (!voxel_checked && !cam_model_.isPointInView(recovered_pos)) {
          // If not in frustum, don't count contributions from this point.
          voxel_checked = true;
        }

        // This is as far as we need to go if this ray is already occluded.
        if (ray_occluded) {
          if (!voxel_checked) {
            occluded_ray++;
            checked_voxels++;
            checked_voxels_set[block_index_ray].insert(voxel_index_ray);
          }
          continue;
        }

        // Otherwise look up this voxel and add it to checked.
        const voxblox::Block<voxblox::TsdfVoxel>::Ptr block_ptr =
            tsdf_layer_->getBlockPtrByIndex(block_index_ray);
        if (block_ptr) {
          // If this block exists, get the voxel.
          const voxblox::TsdfVoxel& voxel =
              block_ptr->getVoxelByVoxelIndex(voxel_index_ray);
          if (voxel.weight > 1e-1 && voxel.distance <= 0.0) {
            if (!voxel_checked) {
              occupied_ray++;
            }
          } 
        } 
        
        if (!voxel_checked) {
          checked_voxels++;
          checked_voxels_set[block_index_ray].insert(voxel_index_ray);
        }
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

  timer_gain.Stop();
  return num_unknown;
}

voxblox::CameraModel& GainEvaluator::getCameraModel() { return cam_model_; }

const voxblox::CameraModel& GainEvaluator::getCameraModel() const {
  return cam_model_;
}
