#include <ros/ros.h>
#include "motion_planning_real_world/AEP/AEP_rw.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "AEP_rw");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    AEP_rw planner(nh, nh_private);
    ros::spin();
    return 0;
}
