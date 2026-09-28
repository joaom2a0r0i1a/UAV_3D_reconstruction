#ifndef GAIN_EVALUATOR_H_
#define GAIN_EVALUATOR_H_

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

#include <ros/ros.h>
#include <ros/package.h>
#include <string>

#include <unordered_set>
#include <unordered_map>
#include <cmath>

// Load an environment region
bool loadEnvironmentRegion(const ros::NodeHandle& nh, const std::string& region,
                           float& min_x, float& max_x, float& min_y, float& max_y, float& min_z, float& max_z);

enum VoxelStatus {kUnknown = 0, kOccupied, kFree};

class GainEvaluator {
  public:
    /*                CONSTRUCTION               */

    GainEvaluator(const ros::NodeHandle& nh_private);

    ~GainEvaluator();

    /*               CONFIGURATION               */

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

    // Real World Box Offset
    void setWorldOffset(const Eigen::Vector3d& offset);

    // Bind the TSDF layer
    void setTsdfLayer(voxblox::Layer<voxblox::TsdfVoxel>* tsdf_layer);

    // Bind the ESDF map
    void setEsdfMap(voxblox::EsdfMap::Ptr esdf_map);

    // Select the scoring objective
    void setObjective(const std::string& objective) {
        objective_ = objective;
    }

    voxblox::CameraModel& getCameraModel();
    const voxblox::CameraModel& getCameraModel() const;

    /*            VOXEL / MAP QUERIES            */

    // Get the voxel status at a given position.
    VoxelStatus getVoxelStatus(const Eigen::Vector3d& position) const;

    // Get the voxel center position.
    void getVoxelCenter(Eigen::Vector3d* center, const Eigen::Vector3d& point);

    // Live map voxel size
    double getVoxelSize() const {
        return dr_;
    }

    // Depth image pixel count
    int depthImagePixels() const {
        int w = (int)std::ceil((2.0f * r_max_ * tanf(fov_y_rad_ * 0.5f)) / dr_);
        int h = (int)std::ceil((2.0f * r_max_ * tanf(fov_p_rad_ * 0.5f)) / dr_);
        return w * h;
    }

    /*          MAP PREP + VISUALIZATION         */

    // Flattens the map within the fixed bounds defined in the GainEvaluator
    std::vector<uint8_t> flattenMap(Eigen::Vector3d& origin_out, Eigen::Vector3i& dim_out);

    // Cache the flattened map on the GPU
    void cacheMapOnGPU(const std::vector<uint8_t>& flat_map, const Eigen::Vector3d& origin, const Eigen::Vector3i& dim);

    // Visualize the flattened GPU map
    sensor_msgs::PointCloud2 visualizeGpuMap(const std::vector<uint8_t>& map, const Eigen::Vector3d& origin, const Eigen::Vector3i& dim);

    // Visualize the camera frustum at a given pose.
    void visualize_frustum(const Eigen::Vector4d& pose, std::vector<geometry_msgs::Point>& points);

    // Initialization for visualization of unknown voxels.
    void visualizeGain(const Eigen::Vector4d& pose, voxblox::Pointcloud& voxels);

    /*          GAIN - CPU, SINGLE POSE          */

    // [Validation] CPU calculation using Flat Map Data
    std::pair<double, double> computeGainCPU_FlatMap(const std::vector<uint8_t>& flat_map, const Eigen::Vector4d& pose, double fixed_yaw = NAN);

    // Gain at the pose yaw
    double computeFixedGainRaycasting(const Eigen::Vector4d& pose, Eigen::Vector3d offset = Eigen::Vector3d::Zero());

    // Gain and best yaw over 360 degrees
    std::pair<double, double> computeGainRaycasting(const Eigen::Vector4d& pose, bool optimize_yaw, const Eigen::Vector3d& offset = Eigen::Vector3d::Zero());

    // Best yaw by frustum gain
    std::pair<double, double> computeGainRaycastingFromSampledYaw(Eigen::Vector4d& position, bool optimize_yaw);

    /*            GAIN - CPU, MARGINAL           */

    // Marginal gain over all ancestors
    std::pair<double, double> computeMarginalGainCPU_AllAncestors(const std::vector<uint8_t>& flat_map, rrt_star::Node* candidate_node, double fixed_yaw = NAN, bool one_parent_only = false, bool commit_observed = false);

    // CPU depth buffer
    std::vector<float> computeDepthBufferCPU(const Eigen::Vector4d& pose, const std::vector<uint8_t>& flat_map, const std::vector<float>& parent_R);

    /*                 GAIN - GPU                */

    // Camera basis rows for a yaw
    std::vector<float> parentCamRows(float yaw) const;

    // Grow the depth pool
    void ensureDepthPool(int n_slots);

    // Batched absolute gain
    std::vector<std::pair<double, double>> computeGainBatchGPU(const std::vector<double>& pos_x, const std::vector<double>& pos_y, const std::vector<double>& pos_z, const std::vector<float>* fixed_yaws = nullptr, float* kernel_ms = nullptr);

    // Reference marginal gain
    void computeMarginalGains(const std::vector<rrt_star::Node*>& nodes, bool optimize_yaw, bool one_parent_only = false);

    // Batched marginal gain
    void computeMarginalGainsBatched(const std::vector<rrt_star::Node*>& nodes, bool optimize_yaw,
                                     bool marginal_split, float& kernel_ms);

    // Absolute gain where unset
    void fillAbsoluteGains(const std::vector<rrt_star::Node*>& nodes, const std::vector<uint8_t>& flat_map,
                           const std::string& eval_compute);

    /*            DISPATCH & REFERENCE           */

    // Config for the unified gain dispatch.
    struct GainConfig {
        // Marginal or Absolute Gain
        bool        marginal_gain;
        // Best Yaw or Node Heading
        bool        optimize_yaw;
        // GPU or CPU
        std::string eval_compute;
        // Split or Fused Kernel
        bool        marginal_split;
        // Also Absolute Gain and Yaw
        bool        track_absolute;
    };

    // Unified gain evaluation
    void evaluateGains(const std::vector<rrt_star::Node*>& nodes, const std::vector<uint8_t>& flat_map,
                       const GainConfig& cfg, float& marg_kernel_ms, float& abs_kernel_ms);

    // Check batched against reference
    std::pair<double, double> checkMarginalBatchedAgainstReference(const std::vector<rrt_star::Node*>& nodes, bool optimize_yaw, bool marginal_split, long& yaw_flips);

    /*                COST & SCORE               */

    // Calculate cost and score
    void computeCost(rrt_star::Node* new_node);

    void computeScore(rrt_star::Node* new_node, double lambda);

    void computeCostTwo(kino_rrt_star::Trajectory* new_trajectory);

    void computeScore(kino_rrt_star::Trajectory* new_trajectory, double lambda1, double lambda2);

    void computeSingleScore(kino_rrt_star::Trajectory* new_trajectory, double lambda1, double lambda2);

  private:
    /*              PRIVATE HELPERS              */

    // Pack a voxel index into a key
    inline uint64_t pack_index(int x, int y, int z);

    // Pack GPU launcher arguments
    GpuMap gpuMap() const;
    GpuSensor gpuSensor() const;

    // Gain sphere angular resolution
    void angularResolution(float& dtheta_rad, float& dphi_rad, int& theta_bins) const;

    // CPU angular parameters
    struct ScanParams {
        float dtheta, dphi, phi_start;
        int theta_bins;
    };
    ScanParams scanParams() const;

    // Pick the FOV yaw window
    std::pair<double, double> pickYawWindow(const std::vector<float>& yaw_gains, float dtheta_rad,
                                            int theta_bins, double fixed_yaw,
                                            int* out_best_idx = nullptr, int* out_sectors = nullptr) const;

    /*                MEMBER STATE               */

    std::string objective_ = "expdecay";

    // NON-OWNED pointer to the tsdf layer to use for evaluating exploration gain.
    voxblox::Layer<voxblox::TsdfVoxel>* tsdf_layer_;
    voxblox::EsdfMap::Ptr esdf_map_;
    voxblox::CameraModel cam_model_;

    // Get map Bounds
    float min_x_, min_y_, min_z_, max_x_, max_y_, max_z_;

    // Region before the takeoff offset, real world only.
    float base_min_x_, base_min_y_, base_min_z_, base_max_x_, base_max_y_, base_max_z_;

    float fov_y_rad_, fov_p_rad_;
    float r_max_;
    float dr_ = 0.2f;
    float camera_pitch_;

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

    // Persistent GPU depth pool
    float* d_depth_pool_ = nullptr;
    int pool_capacity_ = 0;
    int pool_per_ = 0;
};

#endif  // GAIN_EVALUATOR_H_
