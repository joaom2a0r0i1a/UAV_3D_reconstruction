#include <ros/ros.h>
#include "motion_planning_real_world/RH_NBVP/RH_NBVP_rw.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "planner");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    RH_NBVP_rw planner(nh, nh_private);
    ros::spin();
    return 0;
}
