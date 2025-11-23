#include <ros/ros.h>
#include "cache_nodes/geo_cached.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "geo_cached");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    GEOCached geo_cached(nh, nh_private);
    ros::spin();
    return 0;
}
