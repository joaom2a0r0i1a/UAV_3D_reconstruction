# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include;/usr/include/eigen3".split(';') if "${prefix}/include;/usr/include/eigen3" != "" else []
PROJECT_CATKIN_DEPENDS = "rospy;roscpp;mrs_msgs;mrs_lib;std_srvs;visualization_msgs;trajectory_generation;voxblox_ros;comm_msgs;minkindr_conversions;cache_nodes;process_pointcloud;multiagent_collision_check;motion_planning_python".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lNBVMultiPlanner;-lAEPMultiPlanner".split(';') if "-lNBVMultiPlanner;-lAEPMultiPlanner" != "" else []
PROJECT_NAME = "multidrone_motion_planning"
PROJECT_SPACE_DIR = "/usr/local"
PROJECT_VERSION = "0.0.0"
