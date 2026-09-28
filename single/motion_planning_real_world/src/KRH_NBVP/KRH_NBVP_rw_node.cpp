#include <ros/ros.h>
#include "motion_planning_real_world/KRH_NBVP/KRH_NBVP_rw.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "KRH_NBVP_rw");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    KRH_NBVP_rw planner(nh, nh_private);
    ros::spin();
    return 0;
}