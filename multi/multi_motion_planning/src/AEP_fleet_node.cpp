#include <ros/ros.h>
#include "multi_motion_planning/AEP_fleet.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "AEP_fleet");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    AEP_fleet planner(nh, nh_private);
    ros::spin();
    return 0;
}