#include <ros/ros.h>
#include "cache_nodes/cached.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "cached");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    Cached cached(nh, nh_private);
    ros::spin();
    return 0;
}
