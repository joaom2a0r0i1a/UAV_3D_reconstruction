#ifndef VOXBLOX_PLANNING_GAIN_EVALUATOR_H_
#define VOXBLOX_PLANNING_GAIN_EVALUATOR_H_

#include <eth_mav_msgs/eigen_mav_msgs.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/utils/camera_model.h>

#include <cmath>

//namespace mav_planning {

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

  // Set Boundaries
  void setBounds(float& min_x, float& min_y, float& min_z,
                float& max_x, float& max_y, float& max_z, float& gain_range);

  // Initialize the neighbour voxels
  void initialize_neighbour_voxels(float voxel_size);

  // Check if it is a frontier voxel.
  bool isFrontierVoxel(const Eigen::Vector3d& voxel);

  // Use raycasting to discard occluded voxels and compute gain.
  double evaluateExplorationGainWithRaycasting(
      const eth_mav_msgs::EigenTrajectoryPoint& pose, int modulus = 1);

  // Use raycasting to discard occluded voxels, Bircher-style implementation.
  double computeGain(const eth_mav_msgs::EigenTrajectoryPoint& pose, int modulus = 1);

  // Use raycasting to discard occluded voxels and compute gain in neighbours.
  double computeGainInNeighbours(
    const eth_mav_msgs::EigenTrajectoryPoint& pose, int modulus = 1);

  voxblox::CameraModel& getCameraModel();
  const voxblox::CameraModel& getCameraModel() const;

 private:
  // NON-OWNED pointer to the tsdf layer to use for evaluating exploration gain.
  voxblox::Layer<voxblox::TsdfVoxel>* tsdf_layer_;
  voxblox::CameraModel cam_model_;

  bool p_accurate_frontiers_;  // True: explicitely compute all frontier voxels (may degrade performance)
  // false: estimate frontier voxels by checking only some neighbors (detection depends on previous views)
  bool p_surface_frontiers_;  // true: count next to occupied, false: count next
  Eigen::Vector3d neighbor_voxels_[26];

  // Get map Bounds
  float min_x_, min_y_, min_z_, max_x_, max_y_, max_z_;
  float gain_range_; 

  // Cached parameters of the layer.
  float voxel_size_;
  float voxel_size_inv_;
  int voxels_per_side_;
  float voxels_per_side_inv_;
};

//}  // namespace mav_planning

#endif  // VOXBLOX_PLANNING_COMMON_GAIN_EVALUATOR_H_