#include <ros/ros.h>
#include "multi_motion_planning/KRH_NBVP_fleet.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "KRH_NBVP_fleet");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    KRH_NBVP_fleet planner(nh, nh_private);
    ros::spin();
    return 0;
}