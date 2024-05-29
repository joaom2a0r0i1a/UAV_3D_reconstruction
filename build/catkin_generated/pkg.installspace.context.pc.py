# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include;/usr/include/eigen3".split(';') if "${prefix}/include;/usr/include/eigen3" != "" else []
PROJECT_CATKIN_DEPENDS = "rospy;roscpp;mrs_msgs;mrs_lib;std_srvs;visualization_msgs;trajectory_generation;voxblox_ros;minkindr_conversions;cache_nodes".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lNBVPlanner;-lAEPlanner".split(';') if "-lNBVPlanner;-lAEPlanner" != "" else []
PROJECT_NAME = "motion_planning_python"
PROJECT_SPACE_DIR = "/usr/local"
PROJECT_VERSION = "1.0.0"
