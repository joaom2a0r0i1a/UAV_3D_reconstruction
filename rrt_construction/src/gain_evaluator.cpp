#include <ros/ros.h>
#include <eth_trajectory_generation/timing.h>
#include <voxblox/integrator/integrator_utils.h>

#include "rrt_construction/gain_evaluator.h"

extern "C" void wrapper_cuda_malloc(uint8_t** dev_ptr, size_t size);
extern "C" void wrapper_cuda_free(void* dev_ptr);
extern "C" void wrapper_cuda_memcpy(void* dev_ptr, const void* host_ptr, size_t size);

// External CUDA function declaration
extern "C" void launch_aep_kernel_single(
    uint8_t* d_map,
    int dx, int dy, int dz, 
    float ox, float oy, float oz,
    float pos_x, float pos_y, float pos_z,
    float* h_result_gain, float* h_result_yaw,
    float voxel_size, float gain_range, float fov_y, float fov_p, float pitch
);

extern "C" void launch_aep_kernel(
    const uint8_t* h_map, 
    int dx, int dy, int dz, 
    float ox, float oy, float oz,
    float* h_pos_x, float* h_pos_y, float* h_pos_z,
    float* h_results_gain,
    float* h_results_yaw,
    int num_candidates,
    float voxel_size, float gain_range, float fov_y, float fov_p, float pitch
);

extern "C" void launch_aep_kernel_batch(
    uint8_t* d_map,
    int dx, int dy, int dz, 
    float ox, float oy, float oz,
    float* h_pos_x, float* h_pos_y, float* h_pos_z,
    float* h_results_gain,
    float* h_results_yaw,
    int num_candidates,
    float voxel_size, float gain_range, float fov_y, float fov_p, float pitch
);

extern "C" void launch_aep_kernel_batch_depth(
    uint8_t* d_map,
    int dx, int dy, int dz, 
    float ox, float oy, float oz,
    float* h_pos_x, float* h_pos_y, float* h_pos_z,
    float* h_results_gain,
    float* h_results_yaw,
    float* h_results_depths,
    int num_candidates,
    float voxel_size, float gain_range, float fov_y, float fov_p, float pitch
);

extern "C" void launch_marginal_gain_kernel(
    uint8_t* d_map,
    int dx, int dy, int dz,
    float ox, float oy, float oz,
    float h_cand_x, float h_cand_y, float h_cand_z,
    float h_parent_x, float h_parent_y, float h_parent_z,
    float h_parent_yaw, float* h_parent_R, float* h_parent_depth,
    float* h_result_gain, float* h_result_yaw, float* h_result_depths,
    float voxel_size, float gain_range, float fov_y, float fov_p, float pitch
);

extern "C" void launch_marginal_gain_kernel_v2(
    uint8_t* d_map,
    int dx, int dy, int dz,
    float ox, float oy, float oz,
    float h_cand_x, float h_cand_y, float h_cand_z,
    float h_parent_x, float h_parent_y, float h_parent_z,
    float h_parent_yaw, float* h_parent_R, float* h_parent_depth,
    float* h_result_gain, float* h_result_yaw, float* h_result_depths,
    float voxel_size, float gain_range, float fov_y, float fov_p, float pitch
  );

extern "C" void launch_marginal_gain_kernel_v3(
  uint8_t* d_map,
  int dx, int dy, int dz,
  float ox, float oy, float oz,
  float h_cand_x, float h_cand_y, float h_cand_z,
  float h_parent_x, float h_parent_y, float h_parent_z,
  float h_parent_yaw, float* h_parent_R, float* h_parent_depth,
  float* h_result_gain, float* h_result_yaw, float* h_result_depths,
  float voxel_size, float gain_range, float fov_y, float fov_p, float pitch
);

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

  int idx = 0;
  for (int x = -1; x <= 1; x++) {
    for (int y = -1; y <= 1; y++){
      for (int z = -1; z <= 1; z++) {
        if (x == 0 && y == 0 && z == 0) {
          continue;
        }

        if (!p_accurate_frontiers_ && std::abs(x) * std::abs(y) + std::abs(z) > 1) {
          continue;
        }

        c_neighbor_voxels_[idx] = Eigen::Vector3d(x, y, z) * vs;
        idx++;
      }
    }
  }
}

// --- 1. Bit Packing (Infinite Grid Index) ---
inline uint64_t GainEvaluator::pack_index(int x, int y, int z) {
    return ((uint64_t)(x & 0x1FFFFF) << 42) | 
           ((uint64_t)(y & 0x1FFFFF) << 21) | 
           ((uint64_t)(z & 0x1FFFFF));
}

GainEvaluator::~GainEvaluator() {
    if (d_map_) {
        wrapper_cuda_free(d_map_);
        d_map_ = nullptr;
    }
}

/* SETUP FUNCTIONS */

double GainEvaluator::getVerticalFoV(double horizontal_fov, int resolution_x, int resolution_y){
  double aspect_ratio = (double)resolution_x / (double)resolution_y;
  //double vertical_fov = 2.0 * std::atan(std::tan(horizontal_fov / 2.0) / aspect_ratio);
  double vertical_fov = horizontal_fov / aspect_ratio;
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

void GainEvaluator::visualizeGain(const eth_mav_msgs::EigenTrajectoryPoint& pose, voxblox::Pointcloud& voxels) {
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

/* GPU-BASED SETUP FUNCTIONS */

std::vector<uint8_t> GainEvaluator::flattenMap(Eigen::Vector3d& origin_out, Eigen::Vector3i& dim_out) {
    // Setup Bounds
    double width_x = max_x_ - min_x_;
    double width_y = max_y_ - min_y_;
    double width_z = max_z_ - min_z_;

    int dx = std::ceil(width_x * voxel_size_inv_);
    int dy = std::ceil(width_y * voxel_size_inv_);
    int dz = std::ceil(width_z * voxel_size_inv_);

    // Safety check
    if(dx <= 0 || dy <= 0 || dz <= 0) {
        ROS_WARN("Map bounds invalid or zero! Check min/max params.");
        return std::vector<uint8_t>(); 
    }

    // Save outputs for GPU
    origin_out = Eigen::Vector3d(min_x_, min_y_, min_z_);
    dim_out = Eigen::Vector3i(dx, dy, dz);

    // Allocate Grid (Default to UNKNOWN = 2)
    std::vector<uint8_t> grid(dx * dy * dz, 2); 

    // Allocate Status (Serial)
    voxblox::BlockIndexList blocks;
    tsdf_layer_->getAllAllocatedBlocks(&blocks);

    for (const auto& index : blocks) {
        const auto& block = tsdf_layer_->getBlockByIndex(index);
        
        // If the entire block is outside our area of interest, skip it.
        voxblox::Point block_origin = block.origin();
        float b_size = block.block_size();
        
        if (block_origin.x() > max_x_ || block_origin.x() + b_size < min_x_ ||
            block_origin.y() > max_y_ || block_origin.y() + b_size < min_y_ ||
            block_origin.z() > max_z_ || block_origin.z() + b_size < min_z_) {
            continue;
        }

        // Iterate voxels
        for (size_t i = 0; i < block.num_voxels(); ++i) {
            const auto& voxel = block.getVoxelByLinearIndex(i);
            
            // Skip if unobserved (weight is low)
            if (voxel.weight < 1e-6) continue;

            // Get World Position
            voxblox::Point p = block.computeCoordinatesFromLinearIndex(i);
            
            // World -> Grid Index
            int gx = std::floor((p.x() - min_x_) * voxel_size_inv_);
            int gy = std::floor((p.y() - min_y_) * voxel_size_inv_);
            int gz = std::floor((p.z() - min_z_) * voxel_size_inv_);

            // Boundary Check (Strict)
            if (gx >= 0 && gx < dx && gy >= 0 && gy < dy && gz >= 0 && gz < dz) {
                // Flat Index: Z * (Area) + Y * (Width) + X
                int flat_idx = gz * (dx * dy) + gy * dx + gx;

                if (voxel.distance <= voxel_size_) { 
                    grid[flat_idx] = 1; // Occupied
                } else {
                    grid[flat_idx] = 0; // Free
                }
            }
        }
    }

    return grid;
}

void GainEvaluator::cacheMapOnGPU(const std::vector<uint8_t>& flat_map, const Eigen::Vector3d& origin, const Eigen::Vector3i& dim) {
    size_t new_size = flat_map.size() * sizeof(uint8_t);
    if (new_size == 0) return;

    // Only re-allocate if we need more space or map changed size
    if (d_map_ == nullptr || new_size > cached_map_byte_size_) {
        if (d_map_) {
          wrapper_cuda_free(d_map_);
        }
        wrapper_cuda_malloc(&d_map_, new_size);
        cached_map_byte_size_ = new_size;
    }

    // This is the "Expensive" 5ms operation
    wrapper_cuda_memcpy(d_map_, flat_map.data(), new_size);

    // Store metadata for the kernel to use later
    cached_origin_ = origin;
    cached_dim_ = dim;
}

sensor_msgs::PointCloud2 GainEvaluator::visualizeGpuMap(const std::vector<uint8_t>& map, const Eigen::Vector3d& origin, const Eigen::Vector3i& dim) {
    // 1. Create PCL Cloud
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    
    // 2. Iterate (Reversing the flattening logic)
    // CPU Flattener was: z * (dx * dy) + y * dx + x
    int total_voxels = dim.x() * dim.y() * dim.z();
    
    if (map.size() != total_voxels) {
        ROS_ERROR_THROTTLE(1.0, "GPU Map Size Mismatch! Expected %d, Got %lu", total_voxels, map.size());
        return sensor_msgs::PointCloud2();
    }

    for (int z = 0; z < dim.z(); ++z) {
        for (int y = 0; y < dim.y(); ++y) {
            for (int x = 0; x < dim.x(); ++x) {
                
                // Reconstruct Index
                int idx = z * (dim.x() * dim.y()) + y * dim.x() + x;
                uint8_t val = map[idx];

                // Only visualize OCCUPIED voxels (Value 1)
                // If you want to see UNKNOWN, check for val == 2
                if (val != 1) continue; 

                pcl::PointXYZRGB p;
                // Calculate center of the voxel (Origin + Index*Res + HalfRes)
                p.x = origin.x() + (x * dr_) + (dr_ * 0.5); // dr_ is your voxel_size member
                p.y = origin.y() + (y * dr_) + (dr_ * 0.5);
                p.z = origin.z() + (z * dr_) + (dr_ * 0.5);

                // Color: BRIGHT RED for Occupied
                p.r = 255; p.g = 0; p.b = 0;
                cloud.push_back(p);
            }
        }
    }

    // 3. Convert to ROS Message
    sensor_msgs::PointCloud2 msg;
    if (!cloud.empty()) {
        pcl::toROSMsg(cloud, msg);
        msg.header.frame_id = "uav1/world_origin"; // Fixed frame
        msg.header.stamp = ros::Time::now();
    }

    return msg;
}

/* GPU-BASED GAIN COMPUTATION FUNCTIONS */

std::pair<double, double> GainEvaluator::computeGainGPU(const std::vector<double>& pos_x, const std::vector<double>& pos_y, const std::vector<double>& pos_z) {
    // 0. Safety Check
    if (d_map_ == nullptr) {
        ROS_ERROR_THROTTLE(1.0, "[GPU] Map not cached! Call cacheMapOnGPU() first.");
        return {0.0, 0.0};
    }
  
    // 1. Number of Candidates
    int num_candidates = pos_x.size();
    if (num_candidates == 0) return {0.0, 0.0};

    // 2. Data Marshalling (Double -> Float)
    // The GPU prefers floats. We convert the vectors here.
    std::vector<float> x_f(num_candidates);
    std::vector<float> y_f(num_candidates);
    std::vector<float> z_f(num_candidates);

    for(int i=0; i<num_candidates; ++i) {
        x_f[i] = static_cast<float>(pos_x[i]);
        y_f[i] = static_cast<float>(pos_y[i]);
        z_f[i] = static_cast<float>(pos_z[i]);
    }

    // 3. Prepare Output Buffers
    std::vector<float> results_gain(num_candidates);
    std::vector<float> results_yaw(num_candidates);

    // 4. LAUNCH THE KERNEL DIRECTLY
    launch_aep_kernel_batch(
        d_map_, 
        cached_dim_.x(), cached_dim_.y(), cached_dim_.z(),
        (float)cached_origin_.x(), (float)cached_origin_.y(), (float)cached_origin_.z(),
        x_f.data(), y_f.data(), z_f.data(),
        results_gain.data(),
        results_yaw.data(),
        num_candidates,
        (float)dr_, (float)r_max_, (float)fov_y_rad_, (float)fov_p_rad_, (float)(camera_pitch_ * M_PI / 180.0)
    );

    // 5. Return Result
    // For now, we assume the user passed 1 candidate and wants 1 result.
    return { (double)results_gain[0], (double)results_yaw[0] };
}

std::vector<std::pair<double, double>> GainEvaluator::computeGainBatchGPU(const std::vector<double>& pos_x, const std::vector<double>& pos_y, const std::vector<double>& pos_z) {
    // 0. Safety Check
    if (d_map_ == nullptr) {
        ROS_ERROR_THROTTLE(1.0, "[GPU] Map not cached! Call cacheMapOnGPU() first.");
        return {};
    }
  
    int num_candidates = pos_x.size();
    if (num_candidates == 0) return {};

    // 1. Marshall Inputs (Double -> Float)
    std::vector<float> x_f(num_candidates);
    std::vector<float> y_f(num_candidates);
    std::vector<float> z_f(num_candidates);

    for(int i=0; i<num_candidates; ++i) {
        x_f[i] = static_cast<float>(pos_x[i]);
        y_f[i] = static_cast<float>(pos_y[i]);
        z_f[i] = static_cast<float>(pos_z[i]);
    }

    // 2. Output Buffers
    std::vector<float> results_gain(num_candidates);
    std::vector<float> results_yaw(num_candidates);

    // 3. Launch Kernel
    launch_aep_kernel_batch(
        d_map_,
        cached_dim_.x(), cached_dim_.y(), cached_dim_.z(),
        (float)cached_origin_.x(), (float)cached_origin_.y(), (float)cached_origin_.z(),
        x_f.data(), y_f.data(), z_f.data(),
        results_gain.data(), 
        results_yaw.data(),
        num_candidates,
        (float)dr_, (float)r_max_, (float)fov_y_rad_, (float)fov_p_rad_, (float)(camera_pitch_ * M_PI / 180.0)
    );

    /*const float DTHETA_RAD = 2.0f * M_PI / 180.0f;
    const float DPHI_RAD   = 2.0f * M_PI / 180.0f;

    int win_w = (int)((float)fov_y_rad_ / DTHETA_RAD);
    if (win_w < 1) win_w = 1;

    int win_h = (int)((float)fov_p_rad_ / DPHI_RAD);
    if (win_h < 1) win_h = 1;

    int total_floats = num_candidates * win_w * win_h;
    std::vector<float> results_depths(total_floats);

    launch_aep_kernel_batch_depth(
        d_map_,
        cached_dim_.x(), cached_dim_.y(), cached_dim_.z(),
        (float)cached_origin_.x(), (float)cached_origin_.y(), (float)cached_origin_.z(),
        x_f.data(), y_f.data(), z_f.data(),
        results_gain.data(), 
        results_yaw.data(),
        results_depths.data(),
        num_candidates,
        (float)dr_, (float)r_max_, (float)fov_y_rad_, (float)fov_p_rad_, (float)(camera_pitch_ * M_PI / 180.0)
    );*/

    // 4. Return Results
    std::vector<std::pair<double, double>> results;
    results.reserve(num_candidates);
    for(int i=0; i<num_candidates; ++i) {
        results.push_back({ (double)results_gain[i], (double)results_yaw[i] });
    }
    return results;
}

std::pair<double, double> GainEvaluator::computeSingleGainGPU(const double pos_x, const double pos_y, const double pos_z) {
    // 0. Safety Check
    if (d_map_ == nullptr) {
        ROS_ERROR_THROTTLE(1.0, "[GPU] Map not cached! Call cacheMapOnGPU() first.");
        return {0.0, 0.0};
    }

    // 1. Launch The Kernel Wrapper
    // Note: We use the member variables we cached earlier for the parent state
    float results_gain = 0.0f;
    float results_yaw = 0.0f;

    launch_aep_kernel_single(
      d_map_,
      cached_dim_.x(), cached_dim_.y(), cached_dim_.z(),
      (float)cached_origin_.x(), (float)cached_origin_.y(), (float)cached_origin_.z(),
      (float)pos_x, (float)pos_y, (float)pos_z,
      &results_gain, &results_yaw,
      (float)dr_, (float)r_max_, (float)fov_y_rad_, (float)fov_p_rad_, (float)(camera_pitch_ * M_PI / 180.0)
    );

    // 2. Return Result
    // For now, we assume the user passed 1 candidate and wants 1 result.
    return { (double)results_gain, (double)results_yaw };
}

std::pair<double, double> GainEvaluator::computeMarginalGainGPU(const double pos_x, const double pos_y, const double pos_z, const Eigen::Vector3d& parent_pos, const double parent_yaw, std::vector<float>& parent_R, const std::vector<float>& parent_depth, std::vector<float>& result_depths) {
    // 0. Safety Check
    if (d_map_ == nullptr) {
        ROS_ERROR_THROTTLE(1.0, "[GPU] Map not cached! Call cacheMapOnGPU() first.");
        return {0.0, 0.0};
    }

    float dtheta_rad = 2.0f * M_PI / 180.0f;
    float dphi_rad   = 2.0f * M_PI / 180.0f;

    //int window_width  = std::max(1, (int)(fov_y_rad_ / dtheta_rad));
    //int window_height = std::max(1, (int)(fov_p_rad_ / dphi_rad));

    int p_width = ceil((2.0f * r_max_ * tanf(fov_y_rad_ * 0.5f)) / dr_);
    int p_height = ceil((2.0f * r_max_ * tanf(fov_p_rad_ * 0.5f)) / dr_);
    
    // Resize the vector to fit the result
    size_t required_size = p_width * p_height;
    if (result_depths.size() != required_size) {
        result_depths.resize(required_size);
    }
    // 1. Prepare Output Buffers
    float results_gain = 0.0f;
    float results_yaw = 0.0f;

    // 2. Launch The Kernel Wrapper
    // Note: We use the member variables we cached earlier for the parent state
    launch_marginal_gain_kernel(
        d_map_,
        cached_dim_.x(), cached_dim_.y(), cached_dim_.z(),
        (float)cached_origin_.x(), (float)cached_origin_.y(), (float)cached_origin_.z(),
        (float)pos_x, (float)pos_y, (float)pos_z,
        (float)parent_pos.x(), (float)parent_pos.y(), (float)parent_pos.z(),
        (float)parent_yaw, parent_R.data(), (float*)parent_depth.data(),
        &results_gain, &results_yaw, result_depths.data(),
        (float)dr_, (float)r_max_, (float)fov_y_rad_, (float)fov_p_rad_, (float)(camera_pitch_ * M_PI / 180.0)
    );

    // 3. Return Result
    // For now, we assume the user passed 1 candidate and wants 1 result.
    return { (double)results_gain, (double)results_yaw };
}

std::pair<double, double> GainEvaluator::computeMarginalGainGPU_v2(const double pos_x, const double pos_y, const double pos_z, const Eigen::Vector3d& parent_pos, const double parent_yaw, std::vector<float>& parent_R, const std::vector<float>& parent_depth, std::vector<float>& result_depths) {
    // 0. Safety Check
    if (d_map_ == nullptr) {
        ROS_ERROR_THROTTLE(1.0, "[GPU] Map not cached! Call cacheMapOnGPU() first.");
        return {0.0, 0.0};
    }

    float dtheta_rad = 2.0f * M_PI / 180.0f;
    float dphi_rad   = 2.0f * M_PI / 180.0f;

    //int window_width  = std::max(1, (int)(fov_y_rad_ / dtheta_rad));
    //int window_height = std::max(1, (int)(fov_p_rad_ / dphi_rad));

    int p_width = ceil((2.0f * r_max_ * tanf(fov_y_rad_ * 0.5f)) / dr_);
    int p_height = ceil((2.0f * r_max_ * tanf(fov_p_rad_ * 0.5f)) / dr_);
    
    // Resize the vector to fit the result
    size_t required_size = p_width * p_height;
    if (result_depths.size() != required_size) {
        result_depths.resize(required_size);
    }
    // 1. Prepare Output Buffers
    float results_gain = 0.0f;
    float results_yaw = 0.0f;

    // 2. Launch The Kernel Wrapper
    // Note: We use the member variables we cached earlier for the parent state
    launch_marginal_gain_kernel_v2(
        d_map_,
        cached_dim_.x(), cached_dim_.y(), cached_dim_.z(),
        (float)cached_origin_.x(), (float)cached_origin_.y(), (float)cached_origin_.z(),
        (float)pos_x, (float)pos_y, (float)pos_z,
        (float)parent_pos.x(), (float)parent_pos.y(), (float)parent_pos.z(),
        (float)parent_yaw, parent_R.data(), (float*)parent_depth.data(),
        &results_gain, &results_yaw, result_depths.data(),
        (float)dr_, (float)r_max_, (float)fov_y_rad_, (float)fov_p_rad_, (float)(camera_pitch_ * M_PI / 180.0)
    );

    // 3. Return Result
    // For now, we assume the user passed 1 candidate and wants 1 result.
    return { (double)results_gain, (double)results_yaw };
}

std::pair<double, double> GainEvaluator::computeMarginalGainGPU_v3(const double pos_x, const double pos_y, const double pos_z, const Eigen::Vector3d& parent_pos, const double parent_yaw, std::vector<float>& parent_R, const std::vector<float>& parent_depth, std::vector<float>& result_depths) {
    // 0. Safety Check
    if (d_map_ == nullptr) {
        ROS_ERROR_THROTTLE(1.0, "[GPU] Map not cached! Call cacheMapOnGPU() first.");
        return {0.0, 0.0};
    }

    float dtheta_rad = 2.0f * M_PI / 180.0f;
    float dphi_rad   = 2.0f * M_PI / 180.0f;

    //int window_width  = std::max(1, (int)(fov_y_rad_ / dtheta_rad));
    //int window_height = std::max(1, (int)(fov_p_rad_ / dphi_rad));

    int p_width = ceil((2.0f * r_max_ * tanf(fov_y_rad_ * 0.5f)) / dr_);
    int p_height = ceil((2.0f * r_max_ * tanf(fov_p_rad_ * 0.5f)) / dr_);
    
    // Resize the vector to fit the result
    size_t required_size = p_width * p_height;
    if (result_depths.size() != required_size) {
        result_depths.resize(required_size);
    }
    // 1. Prepare Output Buffers
    float results_gain = 0.0f;
    float results_yaw = 0.0f;

    // 2. Launch The Kernel Wrapper
    // Note: We use the member variables we cached earlier for the parent state
    launch_marginal_gain_kernel_v3(
        d_map_,
        cached_dim_.x(), cached_dim_.y(), cached_dim_.z(),
        (float)cached_origin_.x(), (float)cached_origin_.y(), (float)cached_origin_.z(),
        (float)pos_x, (float)pos_y, (float)pos_z,
        (float)parent_pos.x(), (float)parent_pos.y(), (float)parent_pos.z(),
        (float)parent_yaw, parent_R.data(), (float*)parent_depth.data(),
        &results_gain, &results_yaw, result_depths.data(),
        (float)dr_, (float)r_max_, (float)fov_y_rad_, (float)fov_p_rad_, (float)(camera_pitch_ * M_PI / 180.0)
    );

    // 3. Return Result
    // For now, we assume the user passed 1 candidate and wants 1 result.
    return { (double)results_gain, (double)results_yaw };
}

std::vector<float> GainEvaluator::computeDepthBufferCPU(const Eigen::Vector4d& pose, const std::vector<uint8_t>& flat_map, const std::vector<float>& parent_R) {
    // 1. Setup Parameters (Exact match to GPU Wrapper)
    float dr = (float)dr_;
    float r_max = (float)r_max_;
    float fov_y = (float)fov_y_rad_;
    float fov_p = (float)fov_p_rad_;

    // Calculate Dimensions & Intrinsics
    int p_width = ceil((2.0f * r_max_ * tanf(fov_y_rad_ * 0.5f)) / dr_);
    int p_height = ceil((2.0f * r_max_ * tanf(fov_p_rad_ * 0.5f)) / dr_);

    // Intrinsics aligned to (Width - 1)
    float fx = (p_width / 2.0f) / tanf(fov_y * 0.5f);
    float fy = (p_height / 2.0f) / tanf(fov_p * 0.5f);
    float cx = p_width / 2.0f;
    float cy = p_height / 2.0f;

    std::vector<float> cpu_buffer(p_width * p_height);

    // 2. Unpack Rotation Basis Vectors
    float r0x = parent_R[0], r0y = parent_R[1], r0z = parent_R[2];
    float r1x = parent_R[3], r1y = parent_R[4], r1z = parent_R[5];
    float r2x = parent_R[6], r2y = parent_R[7], r2z = parent_R[8];

    // 3. Map Data Accessors
    float ox = (float)cached_origin_.x();
    float oy = (float)cached_origin_.y();
    float oz = (float)cached_origin_.z();
    int dim_x = cached_dim_.x();
    int dim_y = cached_dim_.y();
    int dim_z = cached_dim_.z();
    
    // Parent Position from Vector4d
    float pos_x = (float)pose[0];
    float pos_y = (float)pose[1];
    float pos_z = (float)pose[2];

    // 4. Iterate every pixel
    for (int v = 0; v < p_height; ++v) {
      for (int u = 0; u < p_width; ++u) {
        int global_ray_idx = v * p_width + u;

        // --- A. Unproject Pinhole (Pixel -> Camera Frame) ---
        float x_cam = (u - cx) / fx;
        float y_cam = (v - cy) / fy;
        float z_cam = 1.0f;

        // --- B. Rotate to World using Basis Vectors ---
        // dir = x_cam*R0 + y_cam*R1 + z_cam*R2
        float dir_x = x_cam * r0x + y_cam * r1x + z_cam * r2x;
        float dir_y = x_cam * r0y + y_cam * r1y + z_cam * r2y;
        float dir_z = x_cam * r0z + y_cam * r1z + z_cam * r2z;

        // Normalize
        float norm = sqrtf(dir_x*dir_x + dir_y*dir_y + dir_z*dir_z);
        float inv_norm = 1.0f / norm;
        dir_x *= inv_norm;
        dir_y *= inv_norm;
        dir_z *= inv_norm;

        // --- C. DDA Initialization ---
        float t = 0.0f;
        float final_depth = r_max; 
        float max_t_vox = r_max / dr;

        // Grid Coordinates
        float gx = (pos_x - ox) / dr;
        float gy = (pos_y - oy) / dr;
        float gz = (pos_z - oz) / dr;

        int ix = floor(gx);
        int iy = floor(gy);
        int iz = floor(gz);

        int stepX = (dir_x > 0.0f) ? 1 : ((dir_x < 0.0f) ? -1 : 0);
        int stepY = (dir_y > 0.0f) ? 1 : ((dir_y < 0.0f) ? -1 : 0);
        int stepZ = (dir_z > 0.0f) ? 1 : ((dir_z < 0.0f) ? -1 : 0);

        float tDeltaX = (fabsf(dir_x) > 1e-9f) ? fabsf(1.0f / dir_x) : 1e30f;
        float tDeltaY = (fabsf(dir_y) > 1e-9f) ? fabsf(1.0f / dir_y) : 1e30f;
        float tDeltaZ = (fabsf(dir_z) > 1e-9f) ? fabsf(1.0f / dir_z) : 1e30f;

        float tMaxX = (stepX > 0) ? (ix + 1.0f - gx) * tDeltaX : (gx - ix) * tDeltaX;
        float tMaxY = (stepY > 0) ? (iy + 1.0f - gy) * tDeltaY : (gy - iy) * tDeltaY;
        float tMaxZ = (stepZ > 0) ? (iz + 1.0f - gz) * tDeltaZ : (gz - iz) * tDeltaZ;

        // --- D. DDA Loop ---
        while (t < max_t_vox) {
          if (ix >= 0 && ix < dim_x && iy >= 0 && iy < dim_y && iz >= 0 && iz < dim_z) {
            // Manual Flattening: z * (dx * dy) + y * dx + x
            int flat_idx = iz * (dim_x * dim_y) + iy * dim_x + ix;
            
            // Read from the DOWNLOADED host map
            uint8_t val = flat_map[flat_idx];
            
            if (val == 1) { // V_OCCUPIED
              final_depth = t * dr;
              break;
            }
          }

          if (tMaxX < tMaxY && tMaxX < tMaxZ) {
            ix += stepX; t = tMaxX; tMaxX += tDeltaX;
          } else if (tMaxY < tMaxZ) {
            iy += stepY; t = tMaxY; tMaxY += tDeltaY;
          } else {
            iz += stepZ; t = tMaxZ; tMaxZ += tDeltaZ;
          }
        }

        // --- E. Write Output (Planar Depth) ---
        float dist_sq = x_cam*x_cam + y_cam*y_cam + 1.0f;
        float cos_theta = 1.0f / sqrtf(dist_sq);

        cpu_buffer[global_ray_idx] = final_depth * cos_theta;
    }
  }

  return cpu_buffer;
}

std::pair<double, double> GainEvaluator::computeMarginalGainCPU_HashMap(const std::vector<uint8_t>& flat_map, rrt_star::Node* candidate_node) {
    // --- A. Inherit History ---
    // Start with the set of unknown voxels the parent has already cleared.
    /*if (candidate_node->parent) {
        candidate_node->observed_unknown_voxels = candidate_node->parent->observed_unknown_voxels;
    }*/

    candidate_node->observed_unknown_voxels.clear();

    // --- B. Setup Constants ---
    float voxel_size = voxel_size_;
    float gain_range = r_max_;
    float ox = cached_origin_.x();
    float oy = cached_origin_.y();
    float oz = cached_origin_.z();
    int dim_x = cached_dim_.x();
    int dim_y = cached_dim_.y();
    int dim_z = cached_dim_.z();

    // Camera FoV
    float dtheta_deg = 2.0f;
    float dphi_deg   = 2.0f;
    float dtheta_rad = dtheta_deg * M_PI / 180.0f;
    float dphi_rad   = dphi_deg   * M_PI / 180.0f;

    float fov_p_rad = fov_p_rad_;
    float camera_pitch = camera_pitch_ * M_PI / 180.0f;
    float phi_center = (M_PI / 2.0f) + camera_pitch;
    float phi_start  = phi_center - (fov_p_rad / 2.0f);
    float phi_end    = phi_center + (fov_p_rad / 2.0f);

    int theta_bins = 360 / dtheta_deg;
    std::vector<float> yaw_gains(theta_bins, 0.0f);

    // --- TEMPORARY BUFFER ---
    // Store the voxel keys seen in each yaw bin. 
    // We will only "commit" the keys from the BEST bins to the actual map later.
    std::vector<std::vector<uint64_t>> seen_keys_per_bin(theta_bins);

    // --- C. Raycasting Loop ---
    for (int t_idx = 0; t_idx < theta_bins; ++t_idx) {
        float theta = -M_PI + (t_idx * dtheta_rad);

        for (float phi = phi_start; phi < phi_end; phi += dphi_rad) {
            
            float sin_phi = sin(phi);
            float dir_x = cos(theta) * sin_phi;
            float dir_y = sin(theta) * sin_phi;
            float dir_z = cos(phi);

            // Start Position relative to Grid Origin
            float start_x = candidate_node->point.x();
            float start_y = candidate_node->point.y();
            float start_z = candidate_node->point.z();

            float gx = (start_x - ox) / voxel_size;
            float gy = (start_y - oy) / voxel_size;
            float gz = (start_z - oz) / voxel_size;

            int ix = std::floor(gx);
            int iy = std::floor(gy);
            int iz = std::floor(gz);

            // DDA Initialization
            int stepX = (dir_x > 0) ? 1 : ((dir_x < 0)) ? -1 : 0;
            int stepY = (dir_y > 0) ? 1 : ((dir_y < 0)) ? -1 : 0;
            int stepZ = (dir_z > 0) ? 1 : ((dir_z < 0)) ? -1 : 0;

            float tDeltaX = (dir_x != 0.0f) ? std::abs(1.0f / dir_x) : 1e30f;
            float tDeltaY = (dir_y != 0.0f) ? std::abs(1.0f / dir_y) : 1e30f;
            float tDeltaZ = (dir_z != 0.0f) ? std::abs(1.0f / dir_z) : 1e30f;

            float tMaxX = (stepX > 0) ? (ix + 1.0f - gx) * tDeltaX : (gx - ix) * tDeltaX;
            float tMaxY = (stepY > 0) ? (iy + 1.0f - gy) * tDeltaY : (gy - iy) * tDeltaY;
            float tMaxZ = (stepZ > 0) ? (iz + 1.0f - gz) * tDeltaZ : (gz - iz) * tDeltaZ;

            float ray_gain = 0.0f;
            float t = 0.0f;
            float max_t = gain_range / voxel_size;

            while (t < max_t) {
                // Bounds check against the GLOBAL map size
                if (ix >= 0 && ix < dim_x && iy >= 0 && iy < dim_y && iz >= 0 && iz < dim_z) {
                  int flat_idx = iz * (dim_x * dim_y) + iy * dim_x + ix;
                  uint8_t global_val = flat_map[flat_idx];

                  if (global_val == 1) { 
                      // OCCUPIED (Known Obstacle) -> Stop Ray, Zero Gain
                      break; 
                  } else if (global_val == 2) {                      
                    uint64_t key = pack_index(ix, iy, iz);
                    //candidate_node->observed_unknown_voxels[key] = 1;
                    // 1. Buffer this key for this yaw bin (Used for map update later)
                    seen_keys_per_bin[t_idx].push_back(key);

                    bool parent_saw_it = false;
                    if (candidate_node->parent) {
                      if (candidate_node->parent->observed_unknown_voxels.find(key) != 
                        candidate_node->parent->observed_unknown_voxels.end()) {
                        parent_saw_it = true;
                      }
                    }

                    if (!parent_saw_it) {
                      float t_exit = std::min({tMaxX, tMaxY, tMaxZ});
                      float dt = t_exit - t;
                      float dr = dt * voxel_size_;

                      float r = t * voxel_size_;
                      float term1 = 2.0f * r * r * dr;
                      float term2 = (dr * dr * dr) / 6.0f;
                      ray_gain += (term1 + term2) * dtheta_rad * sin_phi * sin(dphi_rad * 0.5f);
                    }
                  }
                } 

                if (tMaxX < tMaxY && tMaxX < tMaxZ) {
                  ix += stepX;
                  t = tMaxX;
                  tMaxX += tDeltaX;
                } else if (tMaxY < tMaxZ) {
                  iy += stepY;
                  t = tMaxY;
                  tMaxY += tDeltaY;
                } else {
                  iz += stepZ;
                  t = tMaxZ;
                  tMaxZ += tDeltaZ;
                }
              }
            
            if (ray_gain > 0) yaw_gains[t_idx] += ray_gain;
        }
    }

    // --- D. Sliding Window (Select Best Yaw) ---
    float max_gain = 0.0f;
    int best_idx = 0;
    int sectors = (int)(fov_y_rad_ / dtheta_rad);
    if (sectors < 1) sectors = 1;

    for (int i = 0; i < theta_bins; ++i) {
      float current_window = 0.0f;
      for (int k = 0; k < sectors; ++k) {
        int idx = (i + k) % theta_bins;
        current_window += yaw_gains[idx];
      }
      if (current_window > max_gain) {
        max_gain = current_window;
        best_idx = i;
      }
    }

    float start_angle = -M_PI + (best_idx * dtheta_rad);
    float center_angle = start_angle + (fov_y_rad_ * 0.5f);   
    if (center_angle > M_PI) center_angle -= (2.0f * M_PI);

    // --- E. COMMIT HISTORY (Update Map) ---
    // Now we take the voxels from the *chosen* sectors and commit them to the node's history.
    for (int k = 0; k < sectors; ++k) {
      int idx = (best_idx + k) % theta_bins;
      // For every voxel seen in this slice...
      for (uint64_t key : seen_keys_per_bin[idx]) {
        // ...mark it as seen in the candidate node.
        // This ensures children nodes will know this area is cleared.
        candidate_node->observed_unknown_voxels[key] = 1;
      }
    }

    // Save final gain to node
    candidate_node->gain = (double)max_gain;

    return std::make_pair((double)max_gain, (double)center_angle);
}

void GainEvaluator::populateParentHistory(const std::vector<uint8_t>& flat_map, rrt_star::Node* node) {
    if (!node) return;
    
    // 1. Clear existing map to be safe
    node->observed_unknown_voxels.clear();

    // 2. Constants
    float voxel_size = voxel_size_;
    float r_max = r_max_;
    float ox = cached_origin_.x();
    float oy = cached_origin_.y();
    float oz = cached_origin_.z();
    int dim_x = cached_dim_.x();
    int dim_y = cached_dim_.y();
    int dim_z = cached_dim_.z();

    // 3. Angular Parameters
    float dtheta_rad = 2.0f * M_PI / 180.0f; 
    float dphi_rad   = 2.0f * M_PI / 180.0f;
    
    // Vertical FOV (Pitch)
    float fov_p_rad  = fov_p_rad_; 
    float phi_center = (M_PI / 2.0f) + (camera_pitch_ * M_PI / 180.0f);
    float phi_start  = phi_center - (fov_p_rad / 2.0f);
    float phi_end    = phi_center + (fov_p_rad / 2.0f);

    // Horizontal FOV (Yaw) - RESTRICTED TO PARENT'S YAW
    float fov_y_rad = fov_y_rad_;
    
    // Normalize Parent Yaw to [-PI, PI]
    float parent_yaw = node->point[3];
    while (parent_yaw > M_PI) parent_yaw -= 2.0f * M_PI;
    while (parent_yaw <= -M_PI) parent_yaw += 2.0f * M_PI;

    // Calculate Start and End Theta for the Parent's View
    float theta_start = parent_yaw - (fov_y_rad / 2.0f);
    float theta_end   = parent_yaw + (fov_y_rad / 2.0f);

    // 4. Raycasting Loop (Only inside the specific FOV)
    // We iterate using the same step size as the Gain Evaluator to ensure 1:1 matching
    
    for (float theta = theta_start; theta < theta_end; theta += dtheta_rad) {
      for (float phi = phi_start; phi < phi_end; phi += dphi_rad) {
        float sin_phi = sin(phi);
        float dir_x = cos(theta) * sin_phi;
        float dir_y = sin(theta) * sin_phi;
        float dir_z = cos(phi);

        // Start Position
        float start_x = node->point.x();
        float start_y = node->point.y();
        float start_z = node->point.z();

        float gx = (start_x - ox) / voxel_size;
        float gy = (start_y - oy) / voxel_size;
        float gz = (start_z - oz) / voxel_size;

        int ix = std::floor(gx);
        int iy = std::floor(gy);
        int iz = std::floor(gz);

        // DDA Init
        int stepX = (dir_x > 0) ? 1 : ((dir_x < 0)) ? -1 : 0;
        int stepY = (dir_y > 0) ? 1 : ((dir_y < 0)) ? -1 : 0;
        int stepZ = (dir_z > 0) ? 1 : ((dir_z < 0)) ? -1 : 0;
        float tDeltaX = (dir_x != 0.0f) ? std::abs(1.0f / dir_x) : 1e30f;
        float tDeltaY = (dir_y != 0.0f) ? std::abs(1.0f / dir_y) : 1e30f;
        float tDeltaZ = (dir_z != 0.0f) ? std::abs(1.0f / dir_z) : 1e30f;
        float tMaxX = (stepX > 0) ? (ix + 1.0f - gx) * tDeltaX : (gx - ix) * tDeltaX;
        float tMaxY = (stepY > 0) ? (iy + 1.0f - gy) * tDeltaY : (gy - iy) * tDeltaY;
        float tMaxZ = (stepZ > 0) ? (iz + 1.0f - gz) * tDeltaZ : (gz - iz) * tDeltaZ;

        float t = 0.0f;
        float max_t = r_max / voxel_size;

        while (t < max_t) {
          if (ix >= 0 && ix < dim_x && iy >= 0 && iy < dim_y && iz >= 0 && iz < dim_z) {
            int flat_idx = iz * (dim_x * dim_y) + iy * dim_x + ix;
            uint8_t global_val = flat_map[flat_idx];
            if (global_val == 1) { 
              break; 
            } 
            else if (global_val == 2) { 
              uint64_t key = pack_index(ix, iy, iz);
              node->observed_unknown_voxels[key] = 1;
            }
          } 

          // Advance Ray
          if (tMaxX < tMaxY && tMaxX < tMaxZ) {
            ix += stepX; t = tMaxX; tMaxX += tDeltaX;
          } else if (tMaxY < tMaxZ) {
            iy += stepY; t = tMaxY; tMaxY += tDeltaY;
          } else {
            iz += stepZ; t = tMaxZ; tMaxZ += tDeltaZ;
          }
        }
      }
    }
}

std::pair<double, double> GainEvaluator::computeGainCPU_FlatMap(const std::vector<uint8_t>& flat_map, const eth_mav_msgs::EigenTrajectoryPoint& pose) {
  // 1. Setup Constants
  float voxel_size = voxel_size_;
  float gain_range = r_max_;

  float dtheta_deg = 2.0f;
  float dphi_deg   = 2.0f;
  float dtheta_rad = dtheta_deg * M_PI / 180.0f;
  float dphi_rad   = dphi_deg   * M_PI / 180.0f;

  float fov_p_rad = fov_p_rad_;
  float camera_pitch = camera_pitch_ * M_PI / 180.0f;
  float phi_center = (M_PI / 2.0f) + camera_pitch;
  float phi_start  = phi_center - (fov_p_rad / 2.0f);
  float phi_end    = phi_center + (fov_p_rad / 2.0f);

  int theta_bins = 360 / dtheta_deg;
  std::vector<float> yaw_gains(theta_bins, 0.0f);

  // 2. WOO DDA Raycasting Loop
  for (int t_idx = 0; t_idx < theta_bins; ++t_idx) {
    float theta = -M_PI + (t_idx * dtheta_rad);

    for (float phi = phi_start; phi < phi_end; phi += dphi_rad) {
      float sin_phi = sin(phi);
      float dir_x = cos(theta) * sin_phi;
      float dir_y = sin(theta) * sin_phi;
      float dir_z = cos(phi);

      float start_x = pose.position_W.x();
      float start_y = pose.position_W.y();
      float start_z = pose.position_W.z();

      float gx = (start_x - cached_origin_.x()) / voxel_size_;
      float gy = (start_y - cached_origin_.y()) / voxel_size_;
      float gz = (start_z - cached_origin_.z()) / voxel_size_;

      int ix = std::floor(gx);
      int iy = std::floor(gy);
      int iz = std::floor(gz);

      int stepX = (dir_x > 0) ? 1 : ((dir_x < 0)) ? -1 : 0;
      int stepY = (dir_y > 0) ? 1 : ((dir_y < 0)) ? -1 : 0;
      int stepZ = (dir_z > 0) ? 1 : ((dir_z < 0)) ? -1 : 0;

      float tDeltaX = (dir_x != 0.0f) ? std::abs(1.0f / dir_x) : 1e30f;
      float tDeltaY = (dir_y != 0.0f) ? std::abs(1.0f / dir_y) : 1e30f;
      float tDeltaZ = (dir_z != 0.0f) ? std::abs(1.0f / dir_z) : 1e30f;

      float tMaxX, tMaxY, tMaxZ;

      if (stepX > 0) {
        tMaxX = (ix + 1.0f - gx) * tDeltaX;
      } else {
        tMaxX = (gx - ix) * tDeltaX;
      }

      if (stepY > 0) {
        tMaxY = (iy + 1.0f - gy) * tDeltaY;
      } else {
        tMaxY = (gy - iy) * tDeltaY;
      }

      if (stepZ > 0) {
        tMaxZ = (iz + 1.0f - gz) * tDeltaZ;
      } else {
        tMaxZ = (gz - iz) * tDeltaZ;
      }

      float ray_gain = 0.0f;
      float t = 0.0f;
      float max_t = gain_range / voxel_size_;

      while (t < max_t) {
        if (ix >= 0 && ix < cached_dim_.x() &&
          iy >= 0 && iy < cached_dim_.y() &&
          iz >= 0 && iz < cached_dim_.z()) {

          int idx = iz * cached_dim_.x() * cached_dim_.y() + iy * cached_dim_.x() + ix;
          uint8_t val = flat_map[idx];

          if (val == 1) {
            break;
          } else if (val == 2) {
            float t_exit = std::min({tMaxX, tMaxY, tMaxZ});
            float dt = t_exit - t;
            float dr = dt * voxel_size_;

            float r = t * voxel_size_;
            float term1 = 2.0f * r * r * dr;
            float term2 = (dr * dr * dr) / 6.0f;
            ray_gain += (term1 + term2) * dtheta_rad * sin_phi * sin(dphi_rad * 0.5f);
          }
        }

        if (tMaxX < tMaxY && tMaxX < tMaxZ) {
          ix += stepX;
          t = tMaxX;
          tMaxX += tDeltaX;
        } else if (tMaxY < tMaxZ) {
          iy += stepY;
          t = tMaxY;
          tMaxY += tDeltaY;
        } else {
          iz += stepZ;
          t = tMaxZ;
          tMaxZ += tDeltaZ;
        }
      }

      if (ray_gain > 0) {
        yaw_gains[t_idx] += ray_gain;
      }
    }
  }

  // 4. Sliding Window Optimization (Yaw Selection)
  float max_gain = 0.0f;
  int best_idx = 0;

  // How many 10-degree bins fit in the FOV?
  int sectors = (int)(fov_y_rad_ / dtheta_rad);
  if (sectors < 1) sectors = 1;

  for (int i = 0; i < theta_bins; ++i) {
      float current_window = 0.0f;
      for (int k = 0; k < sectors; ++k) {
          int idx = (i + k) % theta_bins;
          current_window += yaw_gains[idx];
      }

      if (current_window > max_gain) {
          max_gain = current_window;
          best_idx = i;
      }
  }

  // Calculate Best Yaw Angle
  float start_angle = -M_PI + (best_idx * dtheta_rad);
  float center_angle = start_angle + (fov_y_rad_ * 0.5f);   

  // Normalize -PI to PI
  if (center_angle > M_PI) center_angle -= (2.0f * M_PI);

  return std::make_pair((double)max_gain, (double)center_angle);
}

std::pair<double, double> GainEvaluator::computeGainCPU_DDA(const std::vector<uint8_t>& flat_map, const eth_mav_msgs::EigenTrajectoryPoint& pose) {
  // 1. Setup Constants
  float voxel_size = voxel_size_;
  float gain_range = r_max_;

  float dtheta_deg = 2.0f;
  float dphi_deg   = 2.0f;
  float dtheta_rad = dtheta_deg * M_PI / 180.0f;
  float dphi_rad   = dphi_deg   * M_PI / 180.0f;

  float fov_p_rad = fov_p_rad_;
  float camera_pitch = camera_pitch_ * M_PI / 180.0f;    
  float phi_center = (M_PI / 2.0f) + camera_pitch;
  float phi_start  = phi_center - (fov_p_rad / 2.0f);
  float phi_end    = phi_center + (fov_p_rad / 2.0f);

  int theta_bins = 360 / dtheta_deg;
  std::vector<float> yaw_gains(theta_bins, 0.0f);

  // 3D DDA Raycasting
  for (int t_idx = 0; t_idx < theta_bins; ++t_idx) {
    float theta = -M_PI + (t_idx * dtheta_rad);

    // Iterate Phi (Elevation)
    for (float phi = phi_start; phi < phi_end; phi += dphi_rad) {
      // Direction Vectors
      float sin_phi = sin(phi);
      float dir_x = cos(theta) * sin_phi;
      float dir_y = sin(theta) * sin_phi;
      float dir_z = cos(phi);

      float p1x = pose.position_W.x();
      float p1y = pose.position_W.y();
      float p1z = pose.position_W.z();

      float p2x = p1x + gain_range * dir_x;
      float p2y = p1y + gain_range * dir_y;
      float p2z = p1z + gain_range * dir_z;

      float dx = p2x - p1x;
      float dy = p2y - p1y;
      float dz = p2z - p1z;

      float step = 0;
      if (std::abs(dx) >= std::abs(dy) && std::abs(dx) >= std::abs(dz)) {
          step = std::abs(dx) * voxel_size_inv_;
      } else if (std::abs(dy) >= std::abs(dx) && std::abs(dy) >= std::abs(dz)) {
          step = std::abs(dy) * voxel_size_inv_;
      } else {
          step = std::abs(dz) * voxel_size_inv_;
      }

      dx /= step;
      dy /= step;
      dz /= step;

      float x = p1x;
      float y = p1y;
      float z = p1z;
      
      int i = 0;
      float ray_gain = 0.0f;
      float dist_per_step = gain_range / step;
      float current_dist = 0.0f;

      while (i <= step) {
        int gx = std::floor((x - cached_origin_.x()) / voxel_size);
        int gy = std::floor((y - cached_origin_.y()) / voxel_size);
        int gz = std::floor((z - cached_origin_.z()) / voxel_size);

        if (gx >= 0 && gx < cached_dim_.x() &&
            gy >= 0 && gy < cached_dim_.y() &&
            gz >= 0 && gz < cached_dim_.z()) {
          int flat_idx = gz * (cached_dim_.x() * cached_dim_.y()) + gy * cached_dim_.x() + gx;
          uint8_t val = flat_map[flat_idx];

          if (val == 1) {
            break;
          } else if (val == 2) {
            float r = current_dist;
            float term1 = 2.0f * r * r * voxel_size;
            float term2 = (voxel_size * voxel_size * voxel_size) / 6.0f;
            float vol = term1 + term2;
            ray_gain += vol * dtheta_rad * sin_phi * sin(dphi_rad / 2.0f);
          }
        }

        x = x + dx;
        y = y + dy;
        z = z + dz;
        current_dist += dist_per_step;
        i = i + 1;
      }

      if (ray_gain > 0) {
        yaw_gains[t_idx] += ray_gain;
      }
    }
  }

  float max_gain = 0.0f;
  int best_idx = 0;

  // How many 10-degree bins fit in the FOV?
  int sectors = (int)(fov_y_rad_ / dtheta_rad);
  if (sectors < 1) sectors = 1;

  for (int i = 0; i < theta_bins; ++i) {
      float current_window = 0.0f;
      for (int k = 0; k < sectors; ++k) {
          int idx = (i + k) % theta_bins;
          current_window += yaw_gains[idx];
      }

      if (current_window > max_gain) {
          max_gain = current_window;
          best_idx = i;
      }
  }

  // Calculate Best Yaw Angle
  float start_angle = -M_PI + (best_idx * dtheta_rad);
  float center_angle = start_angle + (fov_y_rad_ * 0.5f);   

  // Normalize -PI to PI
  if (center_angle > M_PI) center_angle -= (2.0f * M_PI);

  return std::make_pair((double)max_gain, (double)center_angle);
}

std::pair<double, double> GainEvaluator::computeGainCPU_Naive(const std::vector<uint8_t>& flat_map, const eth_mav_msgs::EigenTrajectoryPoint& pose) {
  float voxel_size = voxel_size_;
  float gain_range = r_max_;

  float dtheta_deg = 2.0f;
  float dphi_deg   = 2.0f;
  float dtheta_rad = dtheta_deg * M_PI / 180.0f;
  float dphi_rad   = dphi_deg   * M_PI / 180.0f;

  float fov_p_rad = fov_p_rad_;
  float camera_pitch = camera_pitch_ * M_PI / 180.0f;
  float phi_center = (M_PI / 2.0f) + camera_pitch;
  float phi_start  = phi_center - (fov_p_rad / 2.0f);
  float phi_end    = phi_center + (fov_p_rad / 2.0f);

  int theta_bins = 360 / dtheta_deg;
  std::vector<float> yaw_gains(theta_bins, 0.0f);

    // Original Raycasting Loop 
  for (int t_idx = 0; t_idx < theta_bins; ++t_idx) {
    float theta = -M_PI + (t_idx * dtheta_rad);
    for (float phi = phi_start; phi < phi_end; phi += dphi_rad) {
      // Direction Vectors
      float sin_phi = sin(phi);
      float dir_x = cos(theta) * sin_phi;
      float dir_y = sin(theta) * sin_phi;
      float dir_z = cos(phi);

      float ray_gain = 0.0f;

      // Iterate Ray
      for (float r = 0.2f; r < gain_range; r += voxel_size) {
        // Calculate Point
        float px = pose.position_W.x() + r * dir_x;
        float py = pose.position_W.y() + r * dir_y;
        float pz = pose.position_W.z() + r * dir_z;

        /*// A. STRICT BOUNDARY CHECK (Matches GPU strict fix)
        if (px < min_x_ || px > max_x_ ||
            py < min_y_ || py > max_y_ ||
            pz < min_z_ || pz > max_z_) {
            continue; // Skip out of bounds
        }*/

        // B. CALCULATE INDEX
        int gx = std::floor((px - cached_origin_.x()) / voxel_size);
        int gy = std::floor((py - cached_origin_.y()) / voxel_size);
        int gz = std::floor((pz - cached_origin_.z()) / voxel_size);

        // C. SAFETY CHECK (Should correspond to flat_map size)
        if (gx < 0 || gx >= cached_dim_.x() ||
            gy < 0 || gy >= cached_dim_.y() ||
            gz < 0 || gz >= cached_dim_.z()) {
          continue;
        }

        // D. CHECK VALUE
        int flat_idx = gz * (cached_dim_.x() * cached_dim_.y()) + gy * cached_dim_.x() + gx;
        uint8_t val = flat_map[flat_idx];

        if (val == 1) { // Occupied
          break;
        }
        else if (val == 2) { // Unknown
          // Gain Math (Exact replica of GPU)
          float term1 = 2.0f * r * r * voxel_size;
          float term2 = (voxel_size * voxel_size * voxel_size) / 6.0f;
          float vol = term1 + term2;

          ray_gain += vol * dtheta_rad * sin_phi * sin(dphi_rad / 2.0f);
        }
      }

      // Accumulate to the current Yaw Bin
      if (ray_gain > 0) {
          yaw_gains[t_idx] += ray_gain;
      }
    }
  }

  float max_gain = 0.0f;
  int best_idx = 0;

  // How many 10-degree bins fit in the FOV?
  int sectors = (int)(fov_y_rad_ / dtheta_rad);
  if (sectors < 1) sectors = 1;

  for (int i = 0; i < theta_bins; ++i) {
      float current_window = 0.0f;
      for (int k = 0; k < sectors; ++k) {
          int idx = (i + k) % theta_bins;
          current_window += yaw_gains[idx];
      }

      if (current_window > max_gain) {
          max_gain = current_window;
          best_idx = i;
      }
  }

  // Calculate Best Yaw Angle
  float start_angle = -M_PI + (best_idx * dtheta_rad);
  float center_angle = start_angle + (fov_y_rad_ * 0.5f);   

  // Normalize -PI to PI
  if (center_angle > M_PI) center_angle -= (2.0f * M_PI);

  return std::make_pair((double)max_gain, (double)center_angle);
}

/* GAIN COMPUTATION FUNCTIONS */

double GainEvaluator::computeFixedGainRaycasting(const eth_mav_msgs::EigenTrajectoryPoint& pose) {
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

double GainEvaluator::computeFixedGainRaycasting(const eth_mav_msgs::EigenTrajectoryPoint& pose, Eigen::Vector3d offset) {
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
  //ROS_INFO("[AEPlanner]: RayCasting took: %f seconds.", elapsed.count());
  //std::cout << "AEPGain took " << elapsed.count() << " seconds." << std::endl;

  return gain;
}

std::pair<double, double> GainEvaluator::computeGainRaycasting(const eth_mav_msgs::EigenTrajectoryPoint& pose) {
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

std::pair<double, double> GainEvaluator::computeGainOptimizedRaycasting(const eth_mav_msgs::EigenTrajectoryPoint& pose) {
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

std::pair<double, double> GainEvaluator::computeGainOptimizedRaycasting(const eth_mav_msgs::EigenTrajectoryPoint& pose, Eigen::Vector3d offset) {
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

/* COST AND SCORE FUNCTIONS */

void GainEvaluator::computeCost(rrt_star::Node* new_node) {
    new_node->cost = new_node->parent->cost + (new_node->point.head(3) - new_node->parent->point.head(3)).norm();
}

void GainEvaluator::computeScore(rrt_star::Node* new_node, double lambda) {
    new_node->score = new_node->parent->score + new_node->gain * exp(-lambda * new_node->cost);
}

void GainEvaluator::computeCostTwo(kino_rrt_star::Trajectory* new_trajectory) {
    new_trajectory->cost1 = new_trajectory->parent->cost1 + new_trajectory->cost1;
    new_trajectory->cost2 = new_trajectory->parent->cost2 + new_trajectory->cost2;
}

void GainEvaluator::computeScore(kino_rrt_star::Trajectory* new_trajectory, double lambda1, double lambda2) {
    new_trajectory->score = new_trajectory->parent->score + new_trajectory->gain * exp(-lambda1 * new_trajectory->cost1 - lambda2 * new_trajectory->cost2);
}

void GainEvaluator::computeSingleScore(kino_rrt_star::Trajectory* new_trajectory, double lambda1, double lambda2) {
    new_trajectory->score = new_trajectory->gain * exp(-lambda1 * new_trajectory->cost1 - lambda2 * new_trajectory->cost2);
}

/* GET VOXBLOX CAMERA MODEL */

voxblox::CameraModel& GainEvaluator::getCameraModel() { return cam_model_; }

const voxblox::CameraModel& GainEvaluator::getCameraModel() const {
  return cam_model_;
}
