#include <ros/ros.h>
#include "motion_planning/ImprovedKAEP/ImprovedKAEPlanner.h"
#include <gflags/gflags.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "ImprovedKAEPlanner");
    
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, false);
    google::InstallFailureSignalHandler();

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    ImprovedKAEPlanner improvedkaeplanner(nh, nh_private);
    
    ros::spin();
    return 0;
}