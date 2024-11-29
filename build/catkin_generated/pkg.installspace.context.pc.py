# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include;/usr/include/eigen3".split(';') if "${prefix}/include;/usr/include/eigen3" != "" else []
PROJECT_CATKIN_DEPENDS = "rospy;roscpp;mrs_msgs;mrs_lib;visualization_msgs;trajectory_generation;voxblox_ros;minkindr_conversions;message_generation;geometry_msgs;std_msgs".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lauxiliar".split(';') if "-lauxiliar" != "" else []
PROJECT_NAME = "cache_nodes"
PROJECT_SPACE_DIR = "/usr/local"
PROJECT_VERSION = "0.0.1"
