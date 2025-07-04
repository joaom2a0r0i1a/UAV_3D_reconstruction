#include <ros/ros.h>
#include "motion_planning/KinoNBV/KinoNBVplanner.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "KinoNBVplanner");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    KinoNBVPlanner kinonbvplanner(nh, nh_private);
    ros::spin();
    return 0;
}