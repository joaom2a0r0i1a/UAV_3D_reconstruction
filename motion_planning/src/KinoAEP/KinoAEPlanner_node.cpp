#include <ros/ros.h>
#include "motion_planning/KinoAEP/KinoAEPlanner.h"
#include <gflags/gflags.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "KinoAEPlanner");
    
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, false);
    google::InstallFailureSignalHandler();

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    KinoAEPlanner kinoaeplanner(nh, nh_private);
    
    ros::spin();
    return 0;
}