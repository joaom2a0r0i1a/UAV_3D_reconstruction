#include <ros/ros.h>
#include "motion_planning/KRH_NBVP/KRH_NBVP.h"
#include <gflags/gflags.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "KRH_NBVP");

    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, false);
    google::InstallFailureSignalHandler();

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    KRH_NBVP planner(nh, nh_private);

    ros::spin();
    return 0;
}