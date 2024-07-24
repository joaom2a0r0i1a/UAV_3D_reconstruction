#include <ros/ros.h>
#include "motion_planning_python/NBV/NBVplanner.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "planner");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    NBVPlanner planner(nh, nh_private);
    ros::spin();
    return 0;
}