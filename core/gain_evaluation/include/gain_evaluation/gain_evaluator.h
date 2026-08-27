#ifndef VOXBLOX_PLANNING_GAIN_EVALUATOR_H_
#define VOXBLOX_PLANNING_GAIN_EVALUATOR_H_

#include <Eigen/Dense>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/core/esdf_map.h>
#include <voxblox/utils/camera_model.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <rrt_construction/rrt_star_kd.h>
#include <rrt_construction/kino_rrt_star_kd.h>
#include <gain_evaluation/gpu_raycast_launch.h>

#include <unordered_set>
#include <cmath>
#include <chrono>

enum VoxelStatus {kUnknown = 0, kOccupied, kFree};

class GainEvaluator {
 public:
  GainEvaluator(const ros::NodeHandle& nh_private);

  ~GainEvaluator();

  inline uint64_t pack_index(int x, int y, int z);


  /*              SETUP FUNCTIONS                 */

  // Function to find vertical FoV.  
  double getVerticalFoV(double horizontal_fov, int resolution_x, int resolution_y);

  // Functions to set up the internal camera model.
  void setCameraModelParametersFoV(double horizontal_fov, double vertical_fov,
                                   double min_distance, double max_distance);
  
  // Another function to set up the internal camera model.
  void setCameraModelParametersFocalLength(const Eigen::Vector2d& resolution,
        double focal_length,
        double min_distance,
        double max_distance);

  // Function to set camera extrinsics.
  void setCameraExtrinsics(const voxblox::Transformation& T_C_B);

  // Bind the TSDF layer (NON-OWNED; caller keeps it alive).
  void setTsdfLayer(voxblox::Layer<voxblox::TsdfVoxel>* tsdf_layer);

  // Bind the ESDF map (NON-OWNED; caller keeps it alive).
  void setEsdfMap(voxblox::EsdfMap::Ptr esdf_map);

  // Get the voxel center position.
  void getVoxelCenter(Eigen::Vector3d* center, const Eigen::Vector3d& point);

  // Get the voxel status at a given position.
  VoxelStatus getVoxelStatus(const Eigen::Vector3d& position) const;

  // Visualize the camera frustum at a given pose.
  void visualize_frustum(const Eigen::Vector4d& pose, std::vector<geometry_msgs::Point>& points);

  // Initialization for visualization of unknown voxels.
  void visualizeGain(const Eigen::Vector4d& pose, voxblox::Pointcloud& voxels);


  /*              GPU-BASED SETUP FUNCTIONS                 */

  // Flattens the map within the fixed bounds defined in the GainEvaluator
  std::vector<uint8_t> flattenMap(Eigen::Vector3d& origin_out, Eigen::Vector3i& dim_out);

  // Cache the flattened map on the GPU
  void cacheMapOnGPU(const std::vector<uint8_t>& flat_map, const Eigen::Vector3d& origin, const Eigen::Vector3i& dim);

  // Visualize the flattened GPU map
  sensor_msgs::PointCloud2 visualizeGpuMap(const std::vector<uint8_t>& map, const Eigen::Vector3d& origin, const Eigen::Vector3i& dim);


  /*              GPU-BASED GAIN COMPUTATION FUNCTIONS                 */

  // Batched absolute gain. fixed_yaws (optional): eval the FOV window at each yaw vs optimize. kernel_ms (optional): device time (ms).
  std::vector<std::pair<double, double>> computeGainBatchGPU(const std::vector<double>& pos_x, const std::vector<double>& pos_y, const std::vector<double>& pos_z, const std::vector<float>* fixed_yaws = nullptr, float* kernel_ms = nullptr);

  // Multi-ancestor marginal gain: single-node GPU kernel over the full ancestor chain (traverses observed-free spans, never reseats the DDA).
  std::pair<double, double> computeMultiAncestorMarginalGainGPU(const double pos_x, const double pos_y, const double pos_z, const std::vector<Eigen::Vector3d>& parent_positions, const std::vector<double>& parent_yaws, std::vector<float>& parent_R, const std::vector<float>& parent_depth, std::vector<float>& result_depths, double fixed_yaw = NAN);

  // Depth-image pixel count (p_width*p_height) shared by every ancestor buffer.
  int depthImagePixels() const {
    int w = (int)std::ceil((2.0 * r_max_ * std::tan(fov_y_rad_ * 0.5)) / dr_);
    int h = (int)std::ceil((2.0 * r_max_ * std::tan(fov_p_rad_ * 0.5)) / dr_);
    return w * h;
  }

  // Live map voxel size (= dr_), set from the tsdf layer in setTsdfLayer().
  double getVoxelSize() const { return dr_; }

  // CPU depth-buffer generator (matches the GPU render) — ground truth for GPU depth-accuracy checks.
  std::vector<float> computeDepthBufferCPU(const Eigen::Vector4d& pose, const std::vector<uint8_t>& flat_map, const std::vector<float>& parent_R);

  // fixed_yaw (optional): evaluate the FOV window at that yaw instead of optimizing.
  std::pair<double, double> computeMarginalGainCPU_HashMap(const std::vector<uint8_t>& flat_map, rrt_star::Node* candidate_node, double fixed_yaw = NAN);

  void populateParentHistory(const std::vector<uint8_t>& flat_map, rrt_star::Node* node);

  // All-ancestors CPU marginal gain (one_parent_only = parent only; commit_observed stores the view for descendants, needs shallow-first callers).
  std::pair<double, double> computeMarginalGainCPU_AllAncestors(const std::vector<uint8_t>& flat_map, rrt_star::Node* candidate_node, double fixed_yaw = NAN, bool one_parent_only = false, bool commit_observed = false);


  /*              SHARED GAIN PIPELINE (AEP + RH-NBVP)                 */

  // Per-node camera basis rows (Right, Forward-down-pitched, ...) for a yaw, at the configured camera pitch.
  std::vector<float> parentCamRows(float yaw) const;

  // Batched multi-ancestor marginal gain, GPU-resident-pool (fused/split); kernel time accumulated into kernel_ms.
  void evaluateMarginalGainsBatched(const std::vector<rrt_star::Node*>& nodes, bool optimize_yaw,
                                    bool marginal_split, float& kernel_ms);

  // Grow the persistent depth pool to >= n_slots (preserving contents).
  void ensureDepthPool(int n_slots);

  // Own-view (absolute) gain for nodes whose absolute_gain is unset (< 0), used by AEP global scoring.
  void fillAbsoluteGains(const std::vector<rrt_star::Node*>& nodes, const std::vector<uint8_t>& flat_map,
                         const std::string& eval_compute);

  // Config for the unified gain dispatch.
  struct GainConfig {
    bool        marginal_gain;   // marginal (de-overlapped) vs absolute (own-view)
    bool        optimize_yaw;    // argmax-yaw per node vs evaluate at the node's own heading
    std::string eval_compute;    // "gpu" or "cpu"
    bool        marginal_split;  // marginal+gpu: split vs fused kernel
    bool        track_absolute;  // also populate absolute_gain/absolute_yaw (AEP)
  };

  // Unified gain eval: sets node->gain (+point[3] when optimizing, +absolute_* when tracked); kernel times into marg/abs_kernel_ms.
  void evaluateGains(const std::vector<rrt_star::Node*>& nodes, const std::vector<uint8_t>& flat_map,
                     const GainConfig& cfg, float& marg_kernel_ms, float& abs_kernel_ms);

  // Benchmark correctness: run the batched pool, diff each node's gain vs the layered reference; returns max|dGain|, yaw_flips (out) = yaw disagreements.
  double checkMarginalBatchedAgainstReference(const std::vector<rrt_star::Node*>& nodes, bool optimize_yaw, bool marginal_split, long& yaw_flips);

  // Independent layered reference for one node (own yaws/renders; ignores depth_buffer/point[3]).
  double computeReferenceMarginalGain(rrt_star::Node* node, double& out_yaw, bool one_parent_only = false, bool fixed_mode = false);

  // [Validation] CPU calculation using Flat Map Data
  std::pair<double, double> computeGainCPU_FlatMap(const std::vector<uint8_t>& flat_map, const Eigen::Vector4d& pose, double fixed_yaw = NAN);


  /*              GAIN COMPUTATION FUNCTIONS                 */
  
  // Raycast unknown-voxel volume for the pose's fixed yaw; offset shifts the bounding box (real-world pose offset, default 0).
  double computeFixedGainRaycasting(const Eigen::Vector4d& pose, Eigen::Vector3d offset = Eigen::Vector3d::Zero());

  // 360deg unknown-voxel gain + best yaw. optimize_yaw=false: uniform integer-yaw scan; true: informative search. offset shifts bbox (default 0).
  std::pair<double, double> computeGainRaycasting(const Eigen::Vector4d& pose, bool optimize_yaw, const Eigen::Vector3d& offset = Eigen::Vector3d::Zero());

  // Sample discrete yaws, return the one maximizing frustum gain. optimize_yaw=false: uniform; true: informative coarse-to-fine.
  std::pair<double, double> computeGainRaycastingFromSampledYaw(Eigen::Vector4d& position, bool optimize_yaw);


  /*              COST AND SCORE FUNCTIONS                 */

  // Calculate cost and score
  void computeCost(rrt_star::Node* new_node);

  void computeScore(rrt_star::Node* new_node, double lambda);

  void setObjective(const std::string& objective) { objective_ = objective; }

  void computeCostTwo(kino_rrt_star::Trajectory* new_trajectory);

  void computeScore(kino_rrt_star::Trajectory* new_trajectory, double lambda1, double lambda2);

  void computeSingleScore(kino_rrt_star::Trajectory* new_trajectory, double lambda1, double lambda2);
  

  /*              GET VOXBLOX CAMERA MODEL                 */

  voxblox::CameraModel& getCameraModel();
  const voxblox::CameraModel& getCameraModel() const;

 private:
  std::string objective_ = "expdecay";

  // Bundle the cached map / sensor state into the launcher ABI structs.
  GpuMap gpuMap() const;
  GpuSensor gpuSensor() const;

  // Gain-sphere angular resolution (1 ray/voxel at max range).
  void angularResolution(float& dtheta_rad, float& dphi_rad, int& theta_bins) const;

  // CPU mirror of the GPU make_kernel_params angular block: azimuth/polar step, bin count, pitch-centred phi start.
  struct ScanParams { float dtheta, dphi, phi_start; int theta_bins; };
  ScanParams scanParams() const;

  // Pick the FOV yaw window from the per-bin histogram (mirrors GPU pick_yaw_window); fixed_yaw!=NAN sums at that yaw, else best. Returns {gain, center}.
  std::pair<double, double> pickYawWindow(const std::vector<float>& yaw_gains, float dtheta_rad,
                                          int theta_bins, double fixed_yaw,
                                          int* out_best_idx = nullptr, int* out_sectors = nullptr) const;

  // NON-OWNED pointer to the tsdf layer to use for evaluating exploration gain.
  voxblox::Layer<voxblox::TsdfVoxel>* tsdf_layer_;
  voxblox::Layer<voxblox::EsdfVoxel>* esdf_layer_;
  voxblox::EsdfMap::Ptr esdf_map_;
  voxblox::CameraModel cam_model_;

  // Get map Bounds
  float min_x_, min_y_, min_z_, max_x_, max_y_, max_z_;
  float gain_range_; 
  double fov_y_rad_, fov_p_rad_;
  double r_max_;
  double dr_ = 0.2;
  double camera_pitch_;

  int yaw_samples_;

  // Cached parameters of the layer.
  float voxel_size_;
  float voxel_size_inv_;
  int voxels_per_side_;
  float voxels_per_side_inv_;

  uint8_t* d_map_ = nullptr;
  Eigen::Vector3i cached_dim_;
  Eigen::Vector3d cached_origin_;
  size_t cached_map_byte_size_ = 0;

  // Persistent GPU-resident depth pool (freed in the destructor). Slots are node append indices; slot 0 = root sentinel (-1).
  float* d_depth_pool_ = nullptr;
  int    pool_capacity_ = 0;   // pool size in slots
  int    pool_per_ = 0;        // floats per slot (depth image pixels)
};

#endif  // VOXBLOX_PLANNING_COMMON_GAIN_EVALUATOR_H_