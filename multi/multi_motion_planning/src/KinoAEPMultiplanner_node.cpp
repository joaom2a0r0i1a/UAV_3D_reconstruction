#include <ros/ros.h>
#include "multidrone_motion_planning/KinoAEPMultiplanner.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "KinoAEPMultiPlanner");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    KinoAEPMultiPlanner kinoAEPMultiPlanner(nh, nh_private);
    ros::spin();
    return 0;
}