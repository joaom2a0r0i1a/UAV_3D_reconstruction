#include <ros/ros.h>
#include <voxblox/integrator/integrator_utils.h>

#include <map>
#include <unordered_map>
#include <algorithm>
#include <chrono>

#include "gain_evaluation/gain_evaluator.h"
#include "gain_evaluation/gpu_raycast_launch.h"

// Bundle cached map / sensor state into the launcher ABI structs (grouped args, not a dozen scalars).
GpuMap GainEvaluator::gpuMap() const {
    return GpuMap{d_map_,
                  cached_dim_.x(), cached_dim_.y(), cached_dim_.z(),
                  (float)cached_origin_.x(), (float)cached_origin_.y(), (float)cached_origin_.z()};
}

GpuSensor GainEvaluator::gpuSensor() const {
    return GpuSensor{(float)dr_, (float)r_max_, (float)fov_y_rad_, (float)fov_p_rad_,
                     (float)(camera_pitch_ * M_PI / 180.0)};
}

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
    if (d_depth_pool_) {
        wrapper_depth_pool_free(d_depth_pool_);
        d_depth_pool_ = nullptr;
    }
}

void GainEvaluator::ensureDepthPool(int n_slots) {
    pool_per_ = depthImagePixels();
    wrapper_depth_pool_ensure(&d_depth_pool_, &pool_capacity_, n_slots, pool_per_);
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
  dr_ = voxel_size_;
}

// Gain-sphere angular bins: 180 (2 deg) at the 5 m / 0.2 m reference, finer with r_max and inversely with voxel; mirrors the GPU (CPU==GPU).
void GainEvaluator::angularResolution(float& dtheta_rad, float& dphi_rad, int& theta_bins) const {
  int bins = (int)std::floor(180.0 * (r_max_ / 5.0) * (0.2 / dr_) + 1e-3);
  if (bins < 1) bins = 1;
  if (bins > 640) bins = 640;

  theta_bins = bins;
  dtheta_rad = (float)(2.0 * M_PI / bins);
  dphi_rad = (float)(2.0 * M_PI / bins);
}

// CPU mirror of the GPU make_kernel_params angular block (angularResolution + pitch-centred phi start); same float ops as the originals.
GainEvaluator::ScanParams GainEvaluator::scanParams() const {
  ScanParams s;
  angularResolution(s.dtheta, s.dphi, s.theta_bins);
  float fov_p_rad = fov_p_rad_;
  float camera_pitch = camera_pitch_ * M_PI / 180.0f;
  float phi_center = (M_PI / 2.0f) + camera_pitch;
  s.phi_start = phi_center - (fov_p_rad / 2.0f);
  return s;
}

// Unknown-voxel gain (2 r^2 dr + dr^3/6) * dtheta * sin(phi) * sin(dphi/2); r=range, dr=sub-span. Bit-matches the GPU.
static inline float unknownVoxelGain(float r, float dr, float dtheta_rad, float sin_phi, float dphi_rad) {
  float term1 = 2.0f * r * r * dr;
  float term2 = (dr * dr * dr) / 6.0f;
  return (term1 + term2) * dtheta_rad * sin_phi * sin(dphi_rad * 0.5f);
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

void GainEvaluator::visualize_frustum(const Eigen::Vector4d& pose, std::vector<geometry_msgs::Point>& points) {
  cam_model_.setBodyPose(voxblox::Transformation(
      Eigen::Quaterniond(Eigen::AngleAxisd(pose[3], Eigen::Vector3d::UnitZ())).cast<float>(),
      pose.head<3>().cast<float>()));

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

void GainEvaluator::visualizeGain(const Eigen::Vector4d& pose, voxblox::Pointcloud& voxels) {
  CHECK_NOTNULL(tsdf_layer_);

  cam_model_.setBodyPose(voxblox::Transformation(
      Eigen::Quaterniond(Eigen::AngleAxisd(pose[3], Eigen::Vector3d::UnitZ())).cast<float>(),
      pose.head<3>().cast<float>()));

  // Get the center of the camera to raycast to.
  voxblox::Transformation camera_pose = cam_model_.getCameraPose();
  voxblox::Point camera_center = camera_pose.getPosition();
  double yaw_rad = pose[3];
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
    // Snap the grid origin down to the voxblox voxel grid, else misaligned axes leak rays through walls.
    double ox = std::floor(min_x_ * voxel_size_inv_) * voxel_size_;
    double oy = std::floor(min_y_ * voxel_size_inv_) * voxel_size_;
    double oz = std::floor(min_z_ * voxel_size_inv_) * voxel_size_;

    // Size the grid from the SNAPPED origin so it still covers up to max_* (dims only grow, never shrink).
    int dx = std::ceil((max_x_ - ox) * voxel_size_inv_);
    int dy = std::ceil((max_y_ - oy) * voxel_size_inv_);
    int dz = std::ceil((max_z_ - oz) * voxel_size_inv_);

    // Safety check
    if(dx <= 0 || dy <= 0 || dz <= 0) {
        ROS_WARN("Map bounds invalid or zero! Check min/max params.");
        return std::vector<uint8_t>();
    }

    // Save outputs for GPU
    origin_out = Eigen::Vector3d(ox, oy, oz);
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
            
            // World -> Grid Index (relative to the SNAPPED origin, so cells align with voxblox voxels)
            int gx = std::floor((p.x() - ox) * voxel_size_inv_);
            int gy = std::floor((p.y() - oy) * voxel_size_inv_);
            int gz = std::floor((p.z() - oz) * voxel_size_inv_);

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
    
    // Iterate, reversing the flattening logic (z*(dx*dy) + y*dx + x).
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

                // Only visualize OCCUPIED voxels (val 1; unknown = val 2).
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

std::vector<std::pair<double, double>> GainEvaluator::computeGainBatchGPU(const std::vector<double>& pos_x, const std::vector<double>& pos_y, const std::vector<double>& pos_z, const std::vector<float>* fixed_yaws, float* kernel_ms) {
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
    GpuCandidates cands = {x_f.data(), y_f.data(), z_f.data(), num_candidates};
    GpuResult out = {results_gain.data(), results_yaw.data(), nullptr};
    if (fixed_yaws) launch_absolute_gain_batch_fixed(gpuMap(), cands, out, gpuSensor(), fixed_yaws->data(), kernel_ms);
    else            launch_absolute_gain_batch(gpuMap(), cands, out, gpuSensor(), kernel_ms);

    // 4. Return Results
    std::vector<std::pair<double, double>> results;
    results.reserve(num_candidates);
    for(int i=0; i<num_candidates; ++i) {
        results.push_back({ (double)results_gain[i], (double)results_yaw[i] });
    }
    return results;
}

std::pair<double, double> GainEvaluator::computeMultiAncestorMarginalGainGPU(const double pos_x, const double pos_y, const double pos_z, const std::vector<Eigen::Vector3d>& parent_positions, const std::vector<double>& parent_yaws, std::vector<float>& parent_R, const std::vector<float>& parent_depth, std::vector<float>& result_depths, double fixed_yaw) {
    // Flatten the ancestor chain, run the single-node kernel over all of them (safety check first).
    if (d_map_ == nullptr) {
        ROS_ERROR_THROTTLE(1.0, "[GPU] Map not cached! Call cacheMapOnGPU() first.");
        return {0.0, 0.0};
    }

    int num_ancestors = (int)parent_positions.size();
    if (num_ancestors == 0) {
        // No ancestors -> no marginal subtraction possible; nothing to evaluate against.
        return {0.0, 0.0};
    }

    int p_width = ceil((2.0f * r_max_ * tanf(fov_y_rad_ * 0.5f)) / dr_);
    int p_height = ceil((2.0f * r_max_ * tanf(fov_p_rad_ * 0.5f)) / dr_);

    // Resize the vector to fit the result
    size_t required_size = p_width * p_height;
    if (result_depths.size() != required_size) {
        result_depths.resize(required_size);
    }

    // Flatten per-ancestor pose/yaw into contiguous float arrays matching the kernel layout (x,y,z + one yaw each).
    std::vector<float> parent_pos_flat(3 * num_ancestors);
    std::vector<float> parent_yaw_flat(num_ancestors);
    for (int i = 0; i < num_ancestors; ++i) {
        parent_pos_flat[3*i + 0] = (float)parent_positions[i].x();
        parent_pos_flat[3*i + 1] = (float)parent_positions[i].y();
        parent_pos_flat[3*i + 2] = (float)parent_positions[i].z();
        parent_yaw_flat[i] = (i < (int)parent_yaws.size()) ? (float)parent_yaws[i] : 0.0f;
    }

    // 1. Prepare Output Buffers
    float results_gain = 0.0f;
    float results_yaw = 0.0f;

    // 2. Launch The Kernel Wrapper
    GpuVec3 cand = {(float)pos_x, (float)pos_y, (float)pos_z};
    GpuAncestors ancestors = {num_ancestors,
                              parent_pos_flat.data(),
                              parent_yaw_flat.data(),
                              parent_R.data(),
                              parent_depth.empty() ? nullptr : (float*)parent_depth.data()};
    GpuResult out = {&results_gain, &results_yaw, result_depths.data()};
    if (std::isnan(fixed_yaw)) launch_marginal_gain(gpuMap(), cand, ancestors, out, gpuSensor());
    else launch_marginal_gain_fixed(gpuMap(), cand, ancestors, out, gpuSensor(), (float)fixed_yaw);

    // 3. Return Result
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

// Start bin of the FOV window centred on `yaw` (rad); matches gpuray::window_start_bin_at_yaw for bit-comparable CPU/GPU fixed-yaw gains.
static int cpu_window_start_bin(double yaw, float fov_y_rad, float dtheta_rad, int theta_bins) {
    int start = (int)std::floor(((float)yaw - 0.5f * fov_y_rad + (float)M_PI) / dtheta_rad + 0.5f);
    return ((start % theta_bins) + theta_bins) % theta_bins;
}

// Pick the FOV yaw window from the per-bin histogram (CPU mirror of gpuray::pick_yaw_window): fixed_yaw!=NAN sums at that yaw, else slide for best.
std::pair<double, double> GainEvaluator::pickYawWindow(const std::vector<float>& yaw_gains, float dtheta_rad,
                                                       int theta_bins, double fixed_yaw,
                                                       int* out_best_idx, int* out_sectors) const {
    int sectors = angular_bins(fov_y_rad_, dtheta_rad);
    if (sectors < 1) sectors = 1;
    if (out_sectors) *out_sectors = sectors;

    float max_gain = 0.0f;
    int best_idx = 0;
    if (!std::isnan(fixed_yaw)) {
        best_idx = cpu_window_start_bin(fixed_yaw, fov_y_rad_, dtheta_rad, theta_bins);
        for (int k = 0; k < sectors; ++k) max_gain += yaw_gains[(best_idx + k) % theta_bins];
        if (out_best_idx) *out_best_idx = best_idx;
        return std::make_pair((double)max_gain, fixed_yaw);
    }

    for (int i = 0; i < theta_bins; ++i) {
        float current_window = 0.0f;
        for (int k = 0; k < sectors; ++k) current_window += yaw_gains[(i + k) % theta_bins];
        if (current_window > max_gain) { max_gain = current_window; best_idx = i; }
    }
    if (out_best_idx) *out_best_idx = best_idx;
    float center_angle = -M_PI + (best_idx * dtheta_rad) + (fov_y_rad_ * 0.5f);
    if (center_angle > M_PI) center_angle -= (2.0f * M_PI);
    return std::make_pair((double)max_gain, (double)center_angle);
}

std::pair<double, double> GainEvaluator::computeMarginalGainCPU_HashMap(const std::vector<uint8_t>& flat_map, rrt_star::Node* candidate_node, double fixed_yaw) {
    // Inherit history: start from the unknown voxels the parent already cleared.
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
    float fov_p_rad = fov_p_rad_;
    ScanParams sp = scanParams();
    float dtheta_rad = sp.dtheta, dphi_rad = sp.dphi, phi_start = sp.phi_start;
    int theta_bins = sp.theta_bins;

    std::vector<float> yaw_gains(theta_bins, 0.0f);

    // Per-bin voxel keys; only the best bins get committed to the map later.
    std::vector<std::vector<uint64_t>> seen_keys_per_bin(theta_bins);

    // --- C. Raycasting Loop ---
    for (int t_idx = 0; t_idx < theta_bins; ++t_idx) {
        float theta = -M_PI + (t_idx * dtheta_rad);

        for (int _r = 0, _nr = angular_bins(fov_p_rad, dphi_rad); _r < _nr; ++_r) { float phi = phi_start + _r * dphi_rad;
            
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
                if (ix >= 0 && ix < dim_x && iy >= 0 && iy < dim_y && iz >= 0 && iz < dim_z) {
                  int flat_idx = iz * (dim_x * dim_y) + iy * dim_x + ix;
                  uint8_t global_val = flat_map[flat_idx];

                  if (global_val == 1) {
                      break;
                  } else if (global_val == 2) {
                    uint64_t key = pack_index(ix, iy, iz);
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
                      if (t_exit > max_t) t_exit = max_t;   // cap last voxel at sensor range (match GPU FIX1)
                      float dt = t_exit - t;
                      float dr = dt * voxel_size_;

                      float r = t * voxel_size_;
                      ray_gain += unknownVoxelGain(r, dr, dtheta_rad, sin_phi, dphi_rad);
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

    // --- D. Yaw window: fixed yaw (NBVP) sums the window at that yaw; else slide for the best window. ---
    int sectors = 0, best_idx = 0;
    std::pair<double, double> win = pickYawWindow(yaw_gains, dtheta_rad, theta_bins, fixed_yaw, &best_idx, &sectors);
    double max_gain = win.first;

    // Commit history: store the chosen sectors' voxels into the node's history.
    for (int k = 0; k < sectors; ++k) {
      int idx = (best_idx + k) % theta_bins;
      // For every voxel seen in this slice...
      for (uint64_t key : seen_keys_per_bin[idx]) {
        // Mark seen in the candidate node so children know this area is cleared.
        candidate_node->observed_unknown_voxels[key] = 1;
      }
    }

    // Save final gain to node
    candidate_node->gain = max_gain;

    return win;
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
    float dtheta_rad, dphi_rad; int theta_bins_unused;
    angularResolution(dtheta_rad, dphi_rad, theta_bins_unused);
    
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

    // Raycasting loop over the parent's FOV only, same step size as the gain evaluator (1:1 match).
    
    for (float theta = theta_start; theta < theta_end; theta += dtheta_rad) {
      for (int _r = 0, _nr = angular_bins(fov_p_rad, dphi_rad); _r < _nr; ++_r) { float phi = phi_start + _r * dphi_rad;
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

std::pair<double, double> GainEvaluator::computeMarginalGainCPU_AllAncestors(const std::vector<uint8_t>& flat_map, rrt_star::Node* candidate_node, double fixed_yaw, bool one_parent_only, bool commit_observed) {
    // Marginal gain vs the UNION of all ancestors' observed sets; commit_observed requires shallow-first callers.

    // --- A. Build ancestor union (root..parent) from ancestors' already-committed observed sets. ---
    std::unordered_set<uint64_t> ancestor_union;
    for (rrt_star::Node* a = candidate_node->parent; a != nullptr; a = a->parent) {
      for (const auto& kv : a->observed_unknown_voxels) ancestor_union.insert(kv.first);
      if (one_parent_only) break;   // G_1parent baseline: only the immediate parent
    }

    // --- B. Setup Constants (identical to computeMarginalGainCPU_HashMap). ---
    float voxel_size = voxel_size_;
    float gain_range = r_max_;
    float ox = cached_origin_.x(); float oy = cached_origin_.y(); float oz = cached_origin_.z();
    int dim_x = cached_dim_.x(); int dim_y = cached_dim_.y(); int dim_z = cached_dim_.z();

    float fov_p_rad = fov_p_rad_;
    ScanParams sp = scanParams();
    float dtheta_rad = sp.dtheta, dphi_rad = sp.dphi, phi_start = sp.phi_start;
    int theta_bins = sp.theta_bins;

    std::vector<float> yaw_gains(theta_bins, 0.0f);
    // Per-bin observed voxels (ALL seen, not just marginal) -> committed for the chosen window if requested.
    std::vector<std::vector<uint64_t>> seen_keys_per_bin;
    if (commit_observed) seen_keys_per_bin.resize(theta_bins);

    // --- C. Raycasting Loop: candidate frustum minus the ancestor union. ---
    for (int t_idx = 0; t_idx < theta_bins; ++t_idx) {
      float theta = -M_PI + (t_idx * dtheta_rad);
      for (int _r = 0, _nr = angular_bins(fov_p_rad, dphi_rad); _r < _nr; ++_r) { float phi = phi_start + _r * dphi_rad;
        float sin_phi = sin(phi);
        float dir_x = cos(theta) * sin_phi;
        float dir_y = sin(theta) * sin_phi;
        float dir_z = cos(phi);

        float gx = (candidate_node->point.x() - ox) / voxel_size;
        float gy = (candidate_node->point.y() - oy) / voxel_size;
        float gz = (candidate_node->point.z() - oz) / voxel_size;
        int ix = std::floor(gx); int iy = std::floor(gy); int iz = std::floor(gz);

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
          if (ix >= 0 && ix < dim_x && iy >= 0 && iy < dim_y && iz >= 0 && iz < dim_z) {
            int flat_idx = iz * (dim_x * dim_y) + iy * dim_x + ix;
            uint8_t global_val = flat_map[flat_idx];
            if (global_val == 1) { break; }
            else if (global_val == 2) {
              uint64_t key = pack_index(ix, iy, iz);
              if (commit_observed) seen_keys_per_bin[t_idx].push_back(key);   // all seen (for the commit)
              bool ancestor_saw_it = (ancestor_union.find(key) != ancestor_union.end());
              if (!ancestor_saw_it) {
                float t_exit = std::min({tMaxX, tMaxY, tMaxZ});
                if (t_exit > max_t) t_exit = max_t;
                float dt = t_exit - t;
                float dr = dt * voxel_size_;
                float r = t * voxel_size_;
                ray_gain += unknownVoxelGain(r, dr, dtheta_rad, sin_phi, dphi_rad);
              }
            }
          }
          if (tMaxX < tMaxY && tMaxX < tMaxZ) { ix += stepX; t = tMaxX; tMaxX += tDeltaX; }
          else if (tMaxY < tMaxZ) { iy += stepY; t = tMaxY; tMaxY += tDeltaY; }
          else { iz += stepZ; t = tMaxZ; tMaxZ += tDeltaZ; }
        }
        if (ray_gain > 0) yaw_gains[t_idx] += ray_gain;
      }
    }

    // --- D. Yaw window (identical to HashMap): fixed yaw sums that window, else slide for best. ---
    int sectors = 0, best_idx = 0;
    std::pair<double, double> win = pickYawWindow(yaw_gains, dtheta_rad, theta_bins, fixed_yaw, &best_idx, &sectors);
    double max_gain = win.first;
    // Commit the chosen window's observed voxels so descendants subtract this node's optimized-yaw view (no-op if !commit_observed).
    if (commit_observed) {
      candidate_node->observed_unknown_voxels.clear();
      for (int k = 0; k < sectors; ++k) {
        int idx = (best_idx + k) % theta_bins;
        for (uint64_t key : seen_keys_per_bin[idx]) candidate_node->observed_unknown_voxels[key] = 1;
      }
    }
    return win;
}


/*              SHARED GAIN PIPELINE (AEP + RH-NBVP)                 */

std::vector<float> GainEvaluator::parentCamRows(float yaw) const {
    const float pitch = (float)(camera_pitch_ * M_PI / 180.0);
    float cos_y = cosf(yaw), sin_y = sinf(yaw);
    float cos_p = cosf(pitch), sin_p = sinf(pitch);
    return { sin_y,          -cos_y,          0.0f,
             -sin_p * cos_y, -sin_p * sin_y, -cos_p,
              cos_p * cos_y,  cos_p * sin_y, -sin_p };
}

// Batched marginal gain (pool): ancestor depth read from d_depth_pool_[depth_slot], each render written to its own slot; marginal_split = staged vs fused kernel.
void GainEvaluator::evaluateMarginalGainsBatched(const std::vector<rrt_star::Node*>& nodes,
                                                 bool optimize_yaw, bool marginal_split, float& kernel_ms) {
    kernel_ms = 0.0f;
    if (nodes.empty()) return;

    // Group by tree depth (shallow-first) so a parent's slot is rendered before its children read it.
    std::map<int, std::vector<size_t>> levels;
    int max_slot = 0;
    for (size_t i = 0; i < nodes.size(); ++i) {
        int depth = 0;
        for (rrt_star::Node* a = nodes[i]->parent; a != nullptr; a = a->parent) ++depth;
        levels[depth].push_back(i);
        max_slot = std::max(max_slot, nodes[i]->depth_slot);
        for (rrt_star::Node* a = nodes[i]->parent; a != nullptr; a = a->parent)
            max_slot = std::max(max_slot, a->depth_slot);
    }
    ensureDepthPool(max_slot + 1);   // grow (contents preserved); slot 0 = root sentinel (-1)

    for (const auto& level : levels) {
        const std::vector<size_t>& idxs = level.second;
        const size_t n = idxs.size();

        std::vector<float> cand_x_f(n), cand_y_f(n), cand_z_f(n), fixed_yaws(n);
        std::vector<int>   anc_offsets(1, 0), depth_idx, out_slot(n);
        std::vector<float> anc_pos, anc_yaw, anc_R;
        for (size_t li = 0; li < n; ++li) {
            rrt_star::Node* nd = nodes[idxs[li]];
            cand_x_f[li] = (float)nd->point.x();
            cand_y_f[li] = (float)nd->point.y();
            cand_z_f[li] = (float)nd->point.z();
            fixed_yaws[li] = (float)nd->point[3];
            out_slot[li]   = nd->depth_slot;
            for (rrt_star::Node* a = nd->parent; a != nullptr; a = a->parent) {
                std::vector<float> R_flat = parentCamRows((float)a->point[3]);
                anc_pos.push_back((float)a->point.x());
                anc_pos.push_back((float)a->point.y());
                anc_pos.push_back((float)a->point.z());
                anc_yaw.push_back((float)a->point[3]);
                anc_R.insert(anc_R.end(), R_flat.begin(), R_flat.end());
                depth_idx.push_back(a->depth_slot);   // GLOBAL slot; root -> 0 (pool sentinel -1)
            }
            anc_offsets.push_back((int)(anc_pos.size() / 3));
        }

        int nc = (int)n;
        int total = anc_offsets[nc];
        std::vector<float> gains(nc, 0.0f), yaws(nc, 0.0f);
        GpuCandidates cands = {cand_x_f.data(), cand_y_f.data(), cand_z_f.data(), nc};
        GpuAncestorBatch anc = {nc, anc_offsets.data(), total,
                                anc_pos.data(), anc_yaw.data(), anc_R.data(), depth_idx.data()};
        GpuResult out = {gains.data(), yaws.data(), nullptr};

        float ms = 0.0f;
        const float* fy = optimize_yaw ? nullptr : fixed_yaws.data();
        if (marginal_split)
            launch_marginal_gain_batch_split(gpuMap(), cands, anc, out, gpuSensor(), &ms, fy, d_depth_pool_, out_slot.data());
        else
            launch_marginal_gain_batch_fused(gpuMap(), cands, anc, out, gpuSensor(), &ms, fy, d_depth_pool_, out_slot.data());
        kernel_ms += ms;

        for (size_t li = 0; li < n; ++li) {
            rrt_star::Node* node = nodes[idxs[li]];
            node->gain = gains[li];
            if (optimize_yaw) node->point[3] = yaws[li];
            node->depth_in_pool = true;   // its slot now holds a real render
        }
    }
}

void GainEvaluator::fillAbsoluteGains(const std::vector<rrt_star::Node*>& nodes, const std::vector<uint8_t>& flat_map,
                                      const std::string& eval_compute) {
    std::vector<rrt_star::Node*> todo;
    todo.reserve(nodes.size());
    for (rrt_star::Node* n : nodes) if (n->absolute_gain < 0.0) todo.push_back(n);
    if (todo.empty()) return;

    if (eval_compute == "gpu") {
        std::vector<double> x(todo.size()), y(todo.size()), z(todo.size());
        for (size_t i = 0; i < todo.size(); ++i) { x[i] = todo[i]->point.x(); y[i] = todo[i]->point.y(); z[i] = todo[i]->point.z(); }
        auto res = computeGainBatchGPU(x, y, z);   // own-view (no fixed yaw, no ancestors); map already resident
        for (size_t i = 0; i < todo.size(); ++i) { todo[i]->absolute_gain = res[i].first; todo[i]->absolute_yaw = res[i].second; }
    } else {
        for (size_t i = 0; i < todo.size(); ++i) {
            Eigen::Vector4d p = todo[i]->point;
            auto r = computeGainCPU_FlatMap(flat_map, p);
            todo[i]->absolute_gain = r.first; todo[i]->absolute_yaw = r.second;
        }
    }
}

// Self-contained per-node reference: rebuild root->node layer by layer at each node's own yaw (fixed_mode=point[3], else re-optimized); one_parent_only=G_1parent vs whole-chain G_all. Never touches the pool/depth_buffer.
double GainEvaluator::computeReferenceMarginalGain(rrt_star::Node* node, double& out_yaw, bool one_parent_only, bool fixed_mode) {
    const int per = depthImagePixels();
    std::vector<rrt_star::Node*> chain;
    for (rrt_star::Node* a = node; a != nullptr; a = a->parent) chain.push_back(a);
    std::reverse(chain.begin(), chain.end());   // root first

    std::vector<std::vector<float>> depth(chain.size());   // own render per chain node (empty = unrendered)
    std::vector<double> yaw(chain.size(), 0.0);            // own chosen yaw per chain node
    double gain = 0.0;
    for (size_t k = 1; k < chain.size(); ++k) {            // skip root (chain[0]); layer by layer
        std::vector<Eigen::Vector3d> anc_pos;
        std::vector<double> anc_yaws;
        std::vector<float> anc_R, anc_depth;
        size_t jlo = one_parent_only ? (k - 1) : 0;        // subtraction set: parent only vs whole chain
        for (size_t j = k; j-- > jlo; ) {                  // ancestors near->far: chain[k-1] .. chain[jlo]
            anc_pos.push_back(chain[j]->point.head(3));
            double yj = fixed_mode ? chain[j]->point[3] : yaw[j];
            anc_yaws.push_back(yj);
            std::vector<float> R = parentCamRows((float)yj);
            anc_R.insert(anc_R.end(), R.begin(), R.end());
            // j==0 is the root (never rendered -> unobserved sentinel); j>=1 was rendered in an earlier layer.
            if (j == 0) anc_depth.insert(anc_depth.end(), (size_t)per, -1.0f);
            else        anc_depth.insert(anc_depth.end(), depth[j].begin(), depth[j].end());
        }
        std::vector<float> out_d;
        double cand_fixed = fixed_mode ? chain[k]->point[3] : NAN;
        auto r = computeMultiAncestorMarginalGainGPU(chain[k]->point.x(), chain[k]->point.y(), chain[k]->point.z(),
                                                     anc_pos, anc_yaws, anc_R, anc_depth, out_d, cand_fixed);
        yaw[k] = fixed_mode ? chain[k]->point[3] : r.second; depth[k] = out_d; gain = r.first;
    }
    out_yaw = yaw.empty() ? 0.0 : yaw.back();
    return gain;
}

// Benchmark correctness: run the batched pool, diff each node's gain vs the layered reference; returns max|dGain|, yaw_flips (out) = optimize-yaw disagreements.
double GainEvaluator::checkMarginalBatchedAgainstReference(const std::vector<rrt_star::Node*>& nodes, bool optimize_yaw, bool marginal_split, long& yaw_flips) {
    float ms = 0.0f;
    evaluateMarginalGainsBatched(nodes, optimize_yaw, marginal_split, ms);
    double max_diff = 0.0;
    yaw_flips = 0;
    for (rrt_star::Node* nd : nodes) {
        double pool_gain = nd->gain, pool_yaw = nd->point[3];
        double ref_yaw;
        double ref_gain = computeReferenceMarginalGain(nd, ref_yaw, /*one_parent_only=*/false, /*fixed_mode=*/!optimize_yaw);
        max_diff = std::max(max_diff, std::abs(ref_gain - pool_gain));
        if (optimize_yaw && std::abs(ref_yaw - pool_yaw) > 1e-4) ++yaw_flips;
    }
    return max_diff;
}

void GainEvaluator::evaluateGains(const std::vector<rrt_star::Node*>& nodes, const std::vector<uint8_t>& flat_map,
                                  const GainConfig& cfg, float& marg_kernel_ms, float& abs_kernel_ms) {
    if (nodes.empty()) return;
    const bool gpu = (cfg.eval_compute == "gpu");

    if (cfg.marginal_gain && gpu) {
        // Marginal gain = GPU-resident pool (fused/split). Correctness vs the layered reference is a benchmark-only check.
        evaluateMarginalGainsBatched(nodes, cfg.optimize_yaw, cfg.marginal_split, marg_kernel_ms);
        if (cfg.track_absolute) fillAbsoluteGains(nodes, flat_map, cfg.eval_compute);
    } else if (!cfg.marginal_gain && gpu) {
        std::vector<double> x(nodes.size()), y(nodes.size()), z(nodes.size());
        std::vector<float> fixed_yaws(nodes.size());
        for (size_t i = 0; i < nodes.size(); ++i) {
            x[i] = nodes[i]->point.x(); y[i] = nodes[i]->point.y(); z[i] = nodes[i]->point.z();
            fixed_yaws[i] = (float)nodes[i]->point[3];
        }
        abs_kernel_ms = 0.0f;
        auto res = computeGainBatchGPU(x, y, z, cfg.optimize_yaw ? nullptr : &fixed_yaws, &abs_kernel_ms);
        for (size_t i = 0; i < nodes.size(); ++i) {
            nodes[i]->gain = res[i].first;
            if (cfg.optimize_yaw) nodes[i]->point[3] = res[i].second;
            if (cfg.track_absolute) { nodes[i]->absolute_gain = res[i].first; nodes[i]->absolute_yaw = res[i].second; }
        }
    } else {
        for (rrt_star::Node* nd : nodes) {
            std::pair<double, double> r;
            if (cfg.marginal_gain) {
                if (nd->parent && nd->parent->parent) populateParentHistory(flat_map, nd->parent);
                r = computeMarginalGainCPU_HashMap(flat_map, nd, cfg.optimize_yaw ? NAN : nd->point[3]);
            } else {
                Eigen::Vector4d pose = nd->point;
                r = computeGainCPU_FlatMap(flat_map, pose, cfg.optimize_yaw ? NAN : nd->point[3]);
                if (cfg.track_absolute) { nd->absolute_gain = r.first; nd->absolute_yaw = r.second; }
            }
            nd->gain = r.first;
            if (cfg.optimize_yaw) nd->point[3] = r.second;
        }
        if (cfg.marginal_gain && cfg.track_absolute) fillAbsoluteGains(nodes, flat_map, cfg.eval_compute);
    }
}

std::pair<double, double> GainEvaluator::computeGainCPU_FlatMap(const std::vector<uint8_t>& flat_map, const Eigen::Vector4d& pose, double fixed_yaw) {
  // 1. Setup Constants
  float voxel_size = voxel_size_;
  float gain_range = r_max_;

  float fov_p_rad = fov_p_rad_;
  ScanParams sp = scanParams();
  float dtheta_rad = sp.dtheta, dphi_rad = sp.dphi, phi_start = sp.phi_start;
  int theta_bins = sp.theta_bins;

  std::vector<float> yaw_gains(theta_bins, 0.0f);

  // 2. WOO DDA Raycasting Loop
  for (int t_idx = 0; t_idx < theta_bins; ++t_idx) {
    float theta = -M_PI + (t_idx * dtheta_rad);

    for (int _r = 0, _nr = angular_bins(fov_p_rad, dphi_rad); _r < _nr; ++_r) { float phi = phi_start + _r * dphi_rad;
      float sin_phi = sin(phi);
      float dir_x = cos(theta) * sin_phi;
      float dir_y = sin(theta) * sin_phi;
      float dir_z = cos(phi);

      float start_x = pose.x();
      float start_y = pose.y();
      float start_z = pose.z();

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
            if (t_exit > max_t) t_exit = max_t;   // cap last voxel at sensor range (match GPU FIX1)
            float dt = t_exit - t;
            float dr = dt * voxel_size_;

            float r = t * voxel_size_;
            ray_gain += unknownVoxelGain(r, dr, dtheta_rad, sin_phi, dphi_rad);
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

  // Yaw window: fixed yaw (NBVP) sums the window at that yaw; else slide for the best window (AEP).
  return pickYawWindow(yaw_gains, dtheta_rad, theta_bins, fixed_yaw);
}

/* GAIN COMPUTATION FUNCTIONS */

double GainEvaluator::computeFixedGainRaycasting(const Eigen::Vector4d& pose, Eigen::Vector3d offset) {
  CHECK_NOTNULL(tsdf_layer_);

  cam_model_.setBodyPose(voxblox::Transformation(
      Eigen::Quaterniond(Eigen::AngleAxisd(pose[3], Eigen::Vector3d::UnitZ())).cast<float>(),
      pose.head<3>().cast<float>()));

  // Get the center of the camera to raycast to.
  voxblox::Transformation camera_pose = cam_model_.getCameraPose();
  voxblox::Point camera_center = camera_pose.getPosition();

  double yaw_rad = pose[3];
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
  double min_x = static_cast<double>(min_x_) + offset[0];
  double min_y = static_cast<double>(min_y_) + offset[1];
  double min_z = static_cast<double>(min_z_) + offset[2];
  double max_x = static_cast<double>(max_x_) + offset[0];
  double max_y = static_cast<double>(max_y_) + offset[1];
  double max_z = static_cast<double>(max_z_) + offset[2];

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

  return gain;
}

std::pair<double, double> GainEvaluator::computeGainRaycasting(const Eigen::Vector4d& pose, bool optimize_yaw, const Eigen::Vector3d& offset) {
  CHECK_NOTNULL(tsdf_layer_);

  cam_model_.setBodyPose(voxblox::Transformation(
      Eigen::Quaterniond(Eigen::AngleAxisd(pose[3], Eigen::Vector3d::UnitZ())).cast<float>(),
      pose.head<3>().cast<float>()));

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
  // offset shifts the bounding box (real-world pose initial offset); zero for the default sim case.
  double min_x = static_cast<double>(min_x_) + offset[0];
  double min_y = static_cast<double>(min_y_) + offset[1];
  double min_z = static_cast<double>(min_z_) + offset[2];
  double max_x = static_cast<double>(max_x_) + offset[0];
  double max_y = static_cast<double>(max_y_) + offset[1];
  double max_z = static_cast<double>(max_z_) + offset[2];

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

  double best_yaw = 0;

  if (!optimize_yaw) {
    double best_yaw_score = 0;
    for (int yaw = -180; yaw < 180; yaw++) {
      double yaw_score = 0;
      for (int fov = -fov_y / 2; fov < fov_y / 2; fov++) {
        int theta = yaw + fov;
        if (theta < -180)
          theta += 360;
        if (theta > 180)
          theta -= 360;
        yaw_score += gain_per_yaw[theta];
      }

      if (best_yaw_score < yaw_score) {
        best_yaw_score = yaw_score;
        best_yaw = yaw;
      }
    }
    gain = best_yaw_score;
  } else {
    double best_gain = 0;

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

    // Keep only bins whose neighbourhood could still beat the current best, then refine within them.
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
  }

  double yaw = M_PI * best_yaw / 180.f;

  return std::make_pair(gain, yaw);
}

std::pair<double, double> GainEvaluator::computeGainRaycastingFromSampledYaw(Eigen::Vector4d& position, bool optimize_yaw) {
  double best_gain = 0;
  double best_yaw = 0;

  if (!optimize_yaw) {
    for (int k = 0; k < yaw_samples_; ++k) {
      double yaw = k * 2 * M_PI / yaw_samples_;
      position[3] = yaw;
      double gain = computeFixedGainRaycasting(position);
      if (gain > best_gain) {
        best_gain = gain;
        best_yaw = yaw;
      }
    }
  } else {
    int min_yaw_samples = ceil(2 * M_PI / fov_y_rad_);

    std::vector<double> yaws;
    std::vector<double> gains;
    double min_yaw_step = 2 * M_PI / min_yaw_samples;
    double yaw_step = 2 * M_PI / yaw_samples_;
    int aditional_angles = (yaw_samples_ - min_yaw_samples) / min_yaw_samples;

    for (int k = 0; k < min_yaw_samples; ++k) {
      double yaw = k * min_yaw_step;
      position[3] = yaw;
      double gain = computeFixedGainRaycasting(position);

      yaws.push_back(yaw);
      gains.push_back(gain);

      if (gain > best_gain) {
        best_gain = gain;
        best_yaw = yaw;
      }
    }

    // Keep only bins whose neighbourhood could still beat the current best, then refine within them.
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
        position[3] = yaw;
        double gain = computeFixedGainRaycasting(position);

        if (gain > best_gain) {
          best_gain = gain;
          best_yaw = yaw;
        }
      }
    }
  }

  return std::make_pair(best_gain, best_yaw);
}

/* COST AND SCORE FUNCTIONS */

void GainEvaluator::computeCost(rrt_star::Node* new_node) {
    new_node->cost = new_node->parent->cost + (new_node->point.head(3) - new_node->parent->point.head(3)).norm();
}

void GainEvaluator::computeScore(rrt_star::Node* new_node, double lambda) {
    if (objective_ == "rate_L") {
        new_node->cum_gain = new_node->parent->cum_gain + new_node->gain;
        double L = new_node->cost < 0.1 ? 0.1 : new_node->cost;
        new_node->score = new_node->cum_gain / L;
    } else {
        new_node->score = new_node->parent->score + new_node->gain * exp(-lambda * new_node->cost);
    }
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
