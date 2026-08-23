#include <ros/ros.h>
#include "multidrone_motion_planning/KinoNBVPMultiplanner.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "KinoNBVPMultiPlanner");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    KinoNBVPMultiPlanner kinoNBVPMultiPlanner(nh, nh_private);
    ros::spin();
    return 0;
}