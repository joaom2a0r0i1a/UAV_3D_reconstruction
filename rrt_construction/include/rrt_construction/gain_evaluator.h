#ifndef VOXBLOX_PLANNING_GAIN_EVALUATOR_H_
#define VOXBLOX_PLANNING_GAIN_EVALUATOR_H_

#include <eth_mav_msgs/eigen_mav_msgs.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/core/esdf_map.h>
#include <voxblox/utils/camera_model.h>

#include <rrt_construction/rrt_star_kd.h>
#include <rrt_construction/geo_rrt_kd.h>
#include <rrt_construction/kino_rrt_star_kd.h>
#include <rrt_construction/improved_krrt_kd.h>

#include <cmath>
#include <chrono>

enum VoxelStatus {kUnknown = 0, kOccupied, kFree};

class GainEvaluator {
 public:
  GainEvaluator(const ros::NodeHandle& nh_private);

  // Function to find vertical FoV.  
  double getVerticalFoV(double horizontal_fov, int resolution_x, int resolution_y);

  // Functions to set up the internal camera model.
  void setCameraModelParametersFoV(double horizontal_fov, double vertical_fov,
                                   double min_distance, double max_distance);
  void setCameraModelParametersFocalLength(const Eigen::Vector2d& resolution,
        double focal_length,
        double min_distance,
        double max_distance);
  void setCameraExtrinsics(const voxblox::Transformation& T_C_B);

  bool isPointInView(const voxblox::Point& point, bool first_node) const;

  std::pair<double, double> rayFrustumIntersectionSegment(const voxblox::Point& ray_origin, const voxblox::Point& ray_dir, double min_range, double max_range) const;

  std::vector<std::pair<double, double>> rayMultiFrustumIntersectionSegment(const voxblox::Point& ray_origin, const voxblox::Point& ray_dir, double min_range, double max_range) const;

  bool isFrontierVoxel(const Eigen::Vector3d& voxel);

  // Bind the TSDF layer to one OWNED BY ANOTHER OBJECT. It is up to the user
  // to ensure the layer exists and does not go out of scope.
  void setTsdfLayer(voxblox::Layer<voxblox::TsdfVoxel>* tsdf_layer);

  // Bind the ESDF map to one OWNED BY ANOTHER OBJECT. It is up to the user
  // to ensure the map exists and does not go out of scope.
  void setEsdfMap(voxblox::EsdfMap::Ptr esdf_map);

  void getVoxelCenter(Eigen::Vector3d* center, const Eigen::Vector3d& point);

  VoxelStatus getVoxelStatus(const Eigen::Vector3d& position) const;

  VoxelStatus getVisibility(const Eigen::Vector3d& view_point, const Eigen::Vector3d& voxel_to_test, bool stop_at_unknown_voxel) const;

  void visualize_frustum(const eth_mav_msgs::EigenTrajectoryPoint& pose, std::vector<geometry_msgs::Point>& points);

  // Use raycasting to calculate volume of unknown voxels for fixed angle.
  double computeFixedGainRaycasting(const eth_mav_msgs::EigenTrajectoryPoint& pose, int modulus = 1);

  // Use raycasting to calculate volume of unknown voxels 360 deg around the robot and find angle that
  // corresponds to the biggest gain given the camera frustum, with uniform yaw optimization.
  std::pair<double, double> computeGainRaycasting(const eth_mav_msgs::EigenTrajectoryPoint& pose, int modulus = 1);

  // Use raycasting to calculate volume of unknown voxels 360 deg around the robot and find angle that
  // corresponds to the biggest gain given the camera frustum, with informative yaw optimization.
  std::pair<double, double> computeGainOptimizedRaycasting(const eth_mav_msgs::EigenTrajectoryPoint& pose, int modulus = 1);
  
  // Use raycasting to calculate the volume of unknown voxels visible within a given yaw's camera frustum.
  // Sample multiple discrete yaw angles and select the yaw that maximizes this gain (uniform yaw optimization).
  std::pair<double, double> computeGainRaycastingFromSampledYaw(eth_mav_msgs::EigenTrajectoryPoint& position);
  
  // Use raycasting to calculate the volume of unknown voxels visible within a given yaw's camera frustum.
  // Sample multiple discrete yaw angles and select the yaw that maximizes this gain (informative yaw optimization).
  std::pair<double, double> computeGainRaycastingFromOptimizedSampledYaw(eth_mav_msgs::EigenTrajectoryPoint& position);

  // Use sparse raycasting to calculate volume of unknown voxels for fixed angle (for trajectories).
  double computeGainFixedAngleAEP(const eth_mav_msgs::EigenTrajectoryPoint& previous_pose, const eth_mav_msgs::EigenTrajectoryPoint& pose, bool first_node, int modulus = 1);

  // Use sparse raycasting to calculate volume of unknown voxels for fixed angle.
  double computeGainFixedAngleAEP(const eth_mav_msgs::EigenTrajectoryPoint& pose, int modulus = 1);

  double computeGainFixedAngleAEP(const eth_mav_msgs::EigenTrajectoryPoint& pose, Eigen::Vector3d offset, int modulus = 1);

  // Use sparse raycasting to calculate volume of unknown voxels 360 deg around the robot and find angle that
  // corresponds to the biggest gain given the camera frustum, AEP-style implementation with uniform gain 
  // optimization.
  std::pair<double, double> computeGainAEP(const eth_mav_msgs::EigenTrajectoryPoint& pose, int modulus = 1);
  
  // Use sparse raycasting to calculate volume of unknown voxels 360 deg around the robot and find angle that
  // corresponds to the biggest gain given the camera frustum, AEP-style implementation with informative gain 
  // optimization.
  std::pair<double, double> computeGainOptimizedAEP(const eth_mav_msgs::EigenTrajectoryPoint& pose, int modulus = 1);

  std::tuple<double, double, double> computeGainOptimizedGEO(const eth_mav_msgs::EigenTrajectoryPoint& pose, int modulus = 1);

  std::pair<double, double> computeGainOptimizedAEP(const eth_mav_msgs::EigenTrajectoryPoint& pose, Eigen::Vector3d offset, int modulus = 1);

  std::pair<double, double> computeGainOptimizedwithPrior(const eth_mav_msgs::EigenTrajectoryPoint& previous_pose, const eth_mav_msgs::EigenTrajectoryPoint& pose);

  std::pair<double, double> computeGainOptimizedwithMultiplePriors(const std::vector<eth_mav_msgs::EigenTrajectoryPoint>& previous_poses, const eth_mav_msgs::EigenTrajectoryPoint& pose);

  std::tuple<double, double, double> computeGainOptimizedwithMultiplePriorsMax(const std::vector<eth_mav_msgs::EigenTrajectoryPoint>& previous_poses, const eth_mav_msgs::EigenTrajectoryPoint& pose);

  double computeGainOptimizedwithMultiplePriorsFixed(const std::vector<eth_mav_msgs::EigenTrajectoryPoint>& previous_poses, const eth_mav_msgs::EigenTrajectoryPoint& pose);
  
  // Use sparse raycasting to calculate the volume of unknown voxels visible within a given yaw's camera frustum.
  // Sample multiple discrete yaw angles and select the yaw that maximizes this gain (uniform yaw optimization).
  std::pair<double, double> computeGainFromSampledYawAEP(eth_mav_msgs::EigenTrajectoryPoint& position);

  // Use sparse raycasting to calculate the volume of unknown voxels visible within a given yaw's camera frustum.
  // Sample multiple discrete yaw angles and select the yaw that maximizes this gain (informative yaw optimization).
  std::pair<double, double> computeGainFromOptimizedSampledYawAEP(eth_mav_msgs::EigenTrajectoryPoint& position);

  // Initialization for visualization of unknown voxels.
  void visualizeGainAEP(const eth_mav_msgs::EigenTrajectoryPoint& pose, voxblox::Pointcloud& voxels);

  // Calculate cost and score
  void computeCost(std::shared_ptr<rrt_star::Node>& new_node);

  void computeScore(std::shared_ptr<rrt_star::Node>& new_node, double lambda);

  void computeCost(std::shared_ptr<geo_rrt::Node>& new_node);

  void computeScore(std::shared_ptr<geo_rrt::Node>& new_node, double lambda);
  
  void computeCostTwo(std::shared_ptr<kino_rrt_star::Trajectory>& new_trajectory);

  void computeScore(std::shared_ptr<kino_rrt_star::Trajectory>& new_trajectory, double lambda1, double lambda2);

  void computeSingleScore(std::shared_ptr<kino_rrt_star::Trajectory>& new_trajectory, double lambda1, double lambda2);

  void computeCostTwo(std::shared_ptr<improved_krrt::Node>& new_node);

  void computeScore(std::shared_ptr<improved_krrt::Node>& new_node, double lambda1, double lambda2);

  void computeSingleScore(std::shared_ptr<improved_krrt::Node>& new_node, double lambda1, double lambda2);

  voxblox::CameraModel& getCameraModel();
  const voxblox::CameraModel& getCameraModel() const;

 private:
  // NON-OWNED pointer to the tsdf layer to use for evaluating exploration gain.
  voxblox::Layer<voxblox::TsdfVoxel>* tsdf_layer_;
  voxblox::Layer<voxblox::EsdfVoxel>* esdf_layer_;
  voxblox::EsdfMap::Ptr esdf_map_;

  voxblox::CameraModel prev_cam_model_;
  voxblox::CameraModel cam_model_;

  voxblox::AlignedVector<voxblox::Plane> bounding_planes_;
  std::vector<voxblox::AlignedVector<voxblox::Plane>> frustums;

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
};

#endif  // VOXBLOX_PLANNING_COMMON_GAIN_EVALUATOR_H_