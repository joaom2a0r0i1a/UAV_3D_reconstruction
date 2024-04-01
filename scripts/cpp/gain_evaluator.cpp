//#include <mav_trajectory_generation/timing.h>
#include <voxblox/integrator/integrator_utils.h>

#include "gain_evaluator.h"


//namespace mav_planning {

GainEvaluator::GainEvaluator() {}

void GainEvaluator::setCameraModelParametersFoV(double horizontal_fov, double vertical_fov,
                                                double min_distance, double max_distance) {
  cam_model_.setIntrinsicsFromFoV(horizontal_fov, vertical_fov, min_distance, max_distance);
}

void GainEvaluator::setCameraModelParametersFocalLength(
    const Eigen::Vector2d& resolution, double focal_length, double min_distance,
    double max_distance) {
  cam_model_.setIntrinsicsFromFocalLength(resolution.cast<float>(), focal_length, min_distance, max_distance);
}

void GainEvaluator::setCameraExtrinsics(const voxblox::Transformation& T_C_B) {
  cam_model_.setExtrinsics(T_C_B);
}

/*
void GainEvaluator::setTsdfLayer(voxblox::Layer<voxblox::TsdfVoxel>* tsdf_layer) {
  tsdf_layer_ = tsdf_layer;
  voxel_size_ = tsdf_layer_->voxel_size();
  voxel_size_inv_ = 1.0 / voxel_size_;
  voxels_per_side_ = tsdf_layer_->voxels_per_side();
  voxels_per_side_inv_ = 1.0 / voxels_per_side_;
}

double GainEvaluator::evaluateExplorationGainBircher(
    const mav_msgs::EigenTrajectoryPoint& pose, int modulus) {
  mav_trajectory_generation::timing::Timer timer_gain(
      "exploration/exp_gain_bircher");

  cam_model_.setBodyPose(voxblox::Transformation(
      pose.orientation_W_B.cast<float>(), pose.position_W.cast<float>()));

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
  double gain = 0.0;
  int checked_voxels = 0;
  int voxel_index = 0;
  Eigen::Vector3f pos = aabb_min.cast<float>();
  for (pos.x() = aabb_min.x(); pos.x() < aabb_max.x(); pos.x() += voxel_size_) {
    for (pos.y() = aabb_min.y(); pos.y() < aabb_max.y();
         pos.y() += voxel_size_) {
      for (pos.z() = aabb_min.z(); pos.z() < aabb_max.z();
           pos.z() += voxel_size_) {
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
        // This is a truncating cast, which is I think what we want in this
        // case.
        voxblox::GlobalIndex global_voxel_idx =
            (voxel_size_inv_ * pos).cast<voxblox::LongIndexElement>();
        voxblox::BlockIndex block_index =
            voxblox::getBlockIndexFromGlobalVoxelIndex(global_voxel_idx,
                                                       voxels_per_side_inv_);
        voxblox::VoxelIndex voxel_index = voxblox::getLocalFromGlobalVoxelIndex(
            global_voxel_idx, voxels_per_side_);

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
          const voxblox::GlobalIndex& global_voxel_idx =
              global_voxel_indices[i];
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
            if (voxel.weight > 1e-1 && voxel.distance <= 0.0) {
              // This is an occupied voxel! Mark all the stuff behind
              // it as occluded.
              ray_occluded = true;
              break;
            }
          }
        }
        if (ray_occluded) {
          num_occluded++;
        } else {
          // If it's not occluded, ACTUALLY look up this voxel.
          const voxblox::Block<voxblox::TsdfVoxel>::Ptr block_ptr =
              tsdf_layer_->getBlockPtrByIndex(block_index);
          if (block_ptr) {
            const voxblox::TsdfVoxel& voxel =
                block_ptr->getVoxelByCoordinates(pos);
            if (voxel.weight <= 1e-1) {
              num_unknown++;
            } else if (voxel.distance >= 0.0) {
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
  num_unknown /= checked_voxels;

  return num_unknown;
}

voxblox::CameraModel& GainEvaluator::getCameraModel() { return cam_model_; }

const voxblox::CameraModel& GainEvaluator::getCameraModel() const {
  return cam_model_;
}
*/