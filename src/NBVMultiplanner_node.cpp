#include <ros/ros.h>
#include "multidrone_motion_planning/NBVMultiplanner.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "NBVMultiPlanner");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    NBVMultiPlanner NBVMultiPlanner(nh, nh_private);
    ros::spin();
    return 0;
}