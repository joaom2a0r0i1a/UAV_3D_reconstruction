#ifndef VOXBLOX_PLANNING_GAIN_EVALUATOR_H_
#define VOXBLOX_PLANNING_GAIN_EVALUATOR_H_

#include <eth_mav_msgs/eigen_mav_msgs.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/core/esdf_map.h>
#include <voxblox/utils/camera_model.h>

#include <motion_planning_python/RRT/rrt_star.h>
#include <motion_planning_python/RRT/kino_rrt_star_kd.h>

#include <cmath>
#include <chrono>

//namespace mav_planning {

enum VoxelStatus {kUnknown = 0, kOccupied, kFree};

class GainEvaluator {
 public:
  GainEvaluator();

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

  // Use raycasting to discard occluded voxels and compute gain.
  double evaluateExplorationGainWithRaycasting(
      const eth_mav_msgs::EigenTrajectoryPoint& pose, int modulus = 1);

  double computeFixedGainRaycasting(const eth_mav_msgs::EigenTrajectoryPoint& pose, int modulus = 1);

  std::pair<double, double> computeGainRaycasting(const eth_mav_msgs::EigenTrajectoryPoint& pose, int modulus = 1);

  std::pair<double, double> computeGainOptimizedRaycasting(const eth_mav_msgs::EigenTrajectoryPoint& pose, int modulus = 1);

  std::pair<double, double> computeGainRaycastingFromSampledYaw(eth_mav_msgs::EigenTrajectoryPoint& position);
  
  std::pair<double, double> computeGainRaycastingFromOptimizedSampledYaw(eth_mav_msgs::EigenTrajectoryPoint& position);

  double computeGainFixedAngleAEP(const eth_mav_msgs::EigenTrajectoryPoint& pose, int modulus = 1);

  // Use sparse raycasting to discard occluded voxels, AEP-style implementation.
  std::pair<double, double> computeGainAEP(const eth_mav_msgs::EigenTrajectoryPoint& pose, int modulus = 1);

  std::pair<double, double> computeGainOptimizedAEP(const eth_mav_msgs::EigenTrajectoryPoint& pose, int modulus = 1);

  std::pair<double, double> computeGainFromSampledYawAEP(eth_mav_msgs::EigenTrajectoryPoint& position);

  std::pair<double, double> computeGainFromOptimizedSampledYawAEP(eth_mav_msgs::EigenTrajectoryPoint& position);

  // Initialization for visualization
  void visualizeGainAEP(const eth_mav_msgs::EigenTrajectoryPoint& pose, voxblox::Pointcloud& voxels);
  
  // Use raycasting to discard occluded voxels, Bircher-style implementation.
  double computeGain(const eth_mav_msgs::EigenTrajectoryPoint& pose, int modulus = 1);

  void computeGainFromsampledYaw(const std::shared_ptr<rrt_star::Node>& node, int yaw_samples, eth_mav_msgs::EigenTrajectoryPoint& trajectory_point);

  void computeCost(std::shared_ptr<rrt_star::Node>& new_node);

  void computeScore(std::shared_ptr<rrt_star::Node>& new_node, double lambda);

  void computeCost(std::shared_ptr<kino_rrt_star::Trajectory>& new_trajectory);
  
  void computeCostTwo(std::shared_ptr<kino_rrt_star::Trajectory>& new_trajectory);

  void computeScore(std::shared_ptr<kino_rrt_star::Trajectory>& new_trajectory, double lambda);

  void computeScore(std::shared_ptr<kino_rrt_star::Trajectory>& new_trajectory, double lambda1, double lambda2);

  void computeSingleScore(std::shared_ptr<kino_rrt_star::Trajectory>& new_trajectory, double lambda);

  void computeSingleScore(std::shared_ptr<kino_rrt_star::Trajectory>& new_trajectory, double lambda1, double lambda2);

  voxblox::CameraModel& getCameraModel();
  const voxblox::CameraModel& getCameraModel() const;

 private:
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

  int yaw_samples;

  // Cached parameters of the layer.
  float voxel_size_;
  float voxel_size_inv_;
  int voxels_per_side_;
  float voxels_per_side_inv_;
};

//}  // namespace mav_planning

#endif  // VOXBLOX_PLANNING_COMMON_GAIN_EVALUATOR_H_