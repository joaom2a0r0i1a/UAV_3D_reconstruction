#include <ros/ros.h>
#include "multidrone_motion_planning/AEPMultiplanner.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "AEPMultiPlanner");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    AEPMultiPlanner AEPMultiPlanner(nh, nh_private);
    ros::spin();
    return 0;
}