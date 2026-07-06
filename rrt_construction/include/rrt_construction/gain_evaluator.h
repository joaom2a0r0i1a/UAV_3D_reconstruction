#ifndef VOXBLOX_PLANNING_GAIN_EVALUATOR_H_
#define VOXBLOX_PLANNING_GAIN_EVALUATOR_H_

#include <eth_mav_msgs/eigen_mav_msgs.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/core/esdf_map.h>
#include <voxblox/utils/camera_model.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <rrt_construction/rrt_star_kd.h>
#include <rrt_construction/kino_rrt_star_kd.h>
#include <rrt_construction/gpu_raycast_launch.h>

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

  // Function to check if a point is in the camera frustum.
  bool isPointInView(const voxblox::Point& point, bool first_node) const;

  // Function to check if a voxel is a frontier voxel.
  bool isFrontierVoxel(const Eigen::Vector3d& voxel);

  // Bind the TSDF layer to one OWNED BY ANOTHER OBJECT. It is up to the user
  // to ensure the layer exists and does not go out of scope.
  void setTsdfLayer(voxblox::Layer<voxblox::TsdfVoxel>* tsdf_layer);

  // Bind the ESDF map to one OWNED BY ANOTHER OBJECT. It is up to the user
  // to ensure the map exists and does not go out of scope.
  void setEsdfMap(voxblox::EsdfMap::Ptr esdf_map);

  // Get the voxel center position.
  void getVoxelCenter(Eigen::Vector3d* center, const Eigen::Vector3d& point);

  // Get the voxel status at a given position.
  VoxelStatus getVoxelStatus(const Eigen::Vector3d& position) const;

  // Visualize the camera frustum at a given pose.
  void visualize_frustum(const eth_mav_msgs::EigenTrajectoryPoint& pose, std::vector<geometry_msgs::Point>& points);

  // Initialization for visualization of unknown voxels.
  void visualizeGain(const eth_mav_msgs::EigenTrajectoryPoint& pose, voxblox::Pointcloud& voxels);


  /*              GPU-BASED SETUP FUNCTIONS                 */

  // Flattens the map within the fixed bounds defined in the GainEvaluator
  std::vector<uint8_t> flattenMap(Eigen::Vector3d& origin_out, Eigen::Vector3i& dim_out);

  // Cache the flattened map on the GPU
  void cacheMapOnGPU(const std::vector<uint8_t>& flat_map, const Eigen::Vector3d& origin, const Eigen::Vector3i& dim);

  // Visualize the flattened GPU map
  sensor_msgs::PointCloud2 visualizeGpuMap(const std::vector<uint8_t>& map, const Eigen::Vector3d& origin, const Eigen::Vector3i& dim);


  /*              GPU-BASED GAIN COMPUTATION FUNCTIONS                 */

  // Compute gain for a single pose
  std::pair<double, double> computeGainGPU(const std::vector<double>& pos_x, const std::vector<double>& pos_y, const std::vector<double>& pos_z);

  // Compute gain for a batch of poses
  std::vector<std::pair<double, double>> computeGainBatchGPU(const std::vector<double>& pos_x, const std::vector<double>& pos_y, const std::vector<double>& pos_z);

  std::pair<double, double> computeSingleGainGPU(const double pos_x, const double pos_y, const double pos_z);

  std::pair<double, double> computeMarginalGainGPU(const double pos_x, const double pos_y, const double pos_z, const Eigen::Vector3d& parent_pos, const double parent_yaw, std::vector<float>& parent_R, const std::vector<float>& parent_depth, std::vector<float>& result_depths);

  std::pair<double, double> computeMarginalGainGPU_v2(const double pos_x, const double pos_y, const double pos_z, const Eigen::Vector3d& parent_pos, const double parent_yaw, std::vector<float>& parent_R, const std::vector<float>& parent_depth, std::vector<float>& result_depths);

  std::pair<double, double> computeMarginalGainGPU_v3(const double pos_x, const double pos_y, const double pos_z, const std::vector<Eigen::Vector3d>& parent_positions, const std::vector<double>& parent_yaws, std::vector<float>& parent_R, const std::vector<float>& parent_depth, std::vector<float>& result_depths);

  // v4: same multi-ancestor marginal gain as v3, but the GPU marcher traverses the
  // observed-free spans instead of jumping them (safer, no DDA reseat).
  std::pair<double, double> computeMarginalGainGPU_v4(const double pos_x, const double pos_y, const double pos_z, const std::vector<Eigen::Vector3d>& parent_positions, const std::vector<double>& parent_yaws, std::vector<float>& parent_R, const std::vector<float>& parent_depth, std::vector<float>& result_depths);

  std::vector<float> computeDepthBufferCPU(const Eigen::Vector4d& pose, const std::vector<uint8_t>& flat_map, const std::vector<float>& parent_R);

  std::pair<double, double> computeMarginalGainCPU_HashMap(const std::vector<uint8_t>& flat_map, rrt_star::Node* candidate_node);

  void populateParentHistory(const std::vector<uint8_t>& flat_map, rrt_star::Node* node);

  // [Validation] CPU calculation using Flat Map Data
  std::pair<double, double> computeGainCPU_FlatMap(const std::vector<uint8_t>& flat_map, const eth_mav_msgs::EigenTrajectoryPoint& pose);

  // [Validation] CPU calculation using DDA algorithm
  std::pair<double, double> computeGainCPU_DDA(const std::vector<uint8_t>& flat_map, const eth_mav_msgs::EigenTrajectoryPoint& pose);

  // [Validation] CPU calculation using Naive Raycasting
  std::pair<double, double> computeGainCPU_Naive(const std::vector<uint8_t>& flat_map, const eth_mav_msgs::EigenTrajectoryPoint& pose);


  /*              GAIN COMPUTATION FUNCTIONS                 */
  
  // Use raycasting to calculate volume of unknown voxels for fixed angle.
  double computeFixedGainRaycasting(const eth_mav_msgs::EigenTrajectoryPoint& pose);

  // Use raycasting to calculate volume of unknown voxels for fixed angle (Real-World Experiment Pose Initial Offset).
  double computeFixedGainRaycasting(const eth_mav_msgs::EigenTrajectoryPoint& pose, Eigen::Vector3d offset);

  // Use raycasting to calculate volume of unknown voxels 360 deg around the robot and find angle that
  // corresponds to the biggest gain given the camera frustum, with uniform yaw optimization.
  std::pair<double, double> computeGainRaycasting(const eth_mav_msgs::EigenTrajectoryPoint& pose);

  // Use raycasting to calculate volume of unknown voxels 360 deg around the robot and find angle that
  // corresponds to the biggest gain given the camera frustum, with informative yaw optimization.
  std::pair<double, double> computeGainOptimizedRaycasting(const eth_mav_msgs::EigenTrajectoryPoint& pose);

  // Use raycasting to calculate volume of unknown voxels 360 deg around the robot and find angle that
  // corresponds to the biggest gain given the camera frustum, with informative yaw optimization (Real-World Experiment Pose Initial Offset).
  std::pair<double, double> computeGainOptimizedRaycasting(const eth_mav_msgs::EigenTrajectoryPoint& pose, Eigen::Vector3d offset);
  
  // Use raycasting to calculate the volume of unknown voxels visible within a given yaw's camera frustum.
  // Sample multiple discrete yaw angles and select the yaw that maximizes this gain (uniform yaw optimization).
  std::pair<double, double> computeGainRaycastingFromSampledYaw(eth_mav_msgs::EigenTrajectoryPoint& position);
  
  // Use raycasting to calculate the volume of unknown voxels visible within a given yaw's camera frustum.
  // Sample multiple discrete yaw angles and select the yaw that maximizes this gain (informative yaw optimization).
  std::pair<double, double> computeGainRaycastingFromOptimizedSampledYaw(eth_mav_msgs::EigenTrajectoryPoint& position);


  /*              COST AND SCORE FUNCTIONS                 */

  // Calculate cost and score
  void computeCost(rrt_star::Node* new_node);

  void computeScore(rrt_star::Node* new_node, double lambda);

  void computeCostTwo(kino_rrt_star::Trajectory* new_trajectory);

  void computeScore(kino_rrt_star::Trajectory* new_trajectory, double lambda1, double lambda2);

  void computeSingleScore(kino_rrt_star::Trajectory* new_trajectory, double lambda1, double lambda2);
  

  /*              GET VOXBLOX CAMERA MODEL                 */

  voxblox::CameraModel& getCameraModel();
  const voxblox::CameraModel& getCameraModel() const;

 private:
  // Bundle the cached map / sensor state into the launcher ABI structs.
  GpuMap gpuMap() const;
  GpuSensor gpuSensor() const;

  // NON-OWNED pointer to the tsdf layer to use for evaluating exploration gain.
  voxblox::Layer<voxblox::TsdfVoxel>* tsdf_layer_;
  voxblox::Layer<voxblox::EsdfVoxel>* esdf_layer_;
  voxblox::EsdfMap::Ptr esdf_map_;
  voxblox::CameraModel prev_cam_model_;
  voxblox::CameraModel cam_model_;

  voxblox::AlignedVector<voxblox::Plane> bounding_planes_;

  // Get map Bounds
  float min_x_, min_y_, min_z_, max_x_, max_y_, max_z_;
  float gain_range_; 
  double fov_y_rad_, fov_p_rad_;
  double r_max_;
  double dr_;
  double camera_pitch_;

  int yaw_samples_;

  bool p_accurate_frontiers_;
  Eigen::Vector3d c_neighbor_voxels_[26];
  double p_checking_distance_;

  // Cached parameters of the layer.
  float voxel_size_;
  float voxel_size_inv_;
  int voxels_per_side_;
  float voxels_per_side_inv_;

  uint8_t* d_map_ = nullptr;
  Eigen::Vector3i cached_dim_;
  Eigen::Vector3d cached_origin_;
  size_t cached_map_byte_size_ = 0;
};

#endif  // VOXBLOX_PLANNING_COMMON_GAIN_EVALUATOR_H_