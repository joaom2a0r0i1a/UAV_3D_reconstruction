#include <ros/ros.h>
#include "motion_planning_real_world/KAEP/KAEP_rw.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "KAEP_rw");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    KAEP_rw planner(nh, nh_private);
    ros::spin();
    return 0;
}
