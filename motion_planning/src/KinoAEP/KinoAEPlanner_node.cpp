#include <ros/ros.h>
#include "motion_planning_python/KinoAEP/KinoAEPlanner.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "KinoAEPlanner");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    KinoAEPlanner kinoaeplanner(nh, nh_private);
    ros::spin();
    return 0;
}