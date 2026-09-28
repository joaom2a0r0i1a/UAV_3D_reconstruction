#include <ros/ros.h>
#include "multi_motion_planning/KAEP_fleet.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "KAEP_fleet");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    KAEP_fleet planner(nh, nh_private);
    ros::spin();
    return 0;
}