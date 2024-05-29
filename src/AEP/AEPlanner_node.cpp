#include <ros/ros.h>
#include "AEP/AEPlanner.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "AEPlanner");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    AEPlanner AEPlanner(nh, nh_private);
    ros::spin();
    return 0;
}