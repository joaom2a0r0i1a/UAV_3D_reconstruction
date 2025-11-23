#include <ros/ros.h>
#include "motion_planning/KinoAEP/KinoAEPlanner.h"
#include <gflags/gflags.h>
#include <gperftools/profiler.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "KinoAEPlanner");
    
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, false);
    google::InstallFailureSignalHandler();

    // Start CPU profiler
    //ProfilerStart("/tmp/KAEP_profile_multi_frustum.out");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    KinoAEPlanner kinoaeplanner(nh, nh_private);
    
    ros::spin();

    // Stop CPU profiler
    /*ProfilerStop();
    ROS_INFO("Profiler stopped. Raw data at /tmp/KAEP_profile_multi_frustum.out");

    // Optional: call system command to generate PDF automatically
    std::string command = "google-pprof --pdf ~/motion_workspace/devel/lib/motion_planning/KinoAEP /tmp/KAEP_profile_multi_frustum.out > /tmp/KAEP_profile_multi_frustum.pdf";
    int ret = system(command.c_str());
    if(ret == 0)
        ROS_INFO("Profiling PDF generated: /tmp/KAEP_profile_multi_frustum.pdf");
    else
        ROS_WARN("Failed to generate profiling PDF. Run manually: %s", command.c_str());*/

    return 0;
}