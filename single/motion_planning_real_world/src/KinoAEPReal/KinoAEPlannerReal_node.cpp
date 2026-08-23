#include <ros/ros.h>
#include "motion_planning_real_world/KinoAEPReal/KinoAEPlannerReal.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "KinoAEPlanner");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    KinoAEPlanner kinoaeplanner(nh, nh_private);
    ros::spin();
    return 0;
}
