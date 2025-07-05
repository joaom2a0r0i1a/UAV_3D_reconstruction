#include <ros/ros.h>
#include "motion_planning/NBV/NBVplanner.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "NBVPlanner");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    NBVPlanner nbvplanner(nh, nh_private);
    ros::spin();
    return 0;
}