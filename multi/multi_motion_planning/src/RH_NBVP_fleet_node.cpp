#include <ros/ros.h>
#include "multidrone_motion_planning/RH_NBVP_fleet.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "RH_NBVP_fleet");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    RH_NBVP_fleet planner(nh, nh_private);
    ros::spin();
    return 0;
}