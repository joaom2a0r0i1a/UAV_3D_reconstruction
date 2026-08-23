#include <ros/ros.h>
#include "cache_nodes/cached_multi.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "multi_cached");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    MultiCached multicached(nh, nh_private);
    ros::spin();
    return 0;
}
