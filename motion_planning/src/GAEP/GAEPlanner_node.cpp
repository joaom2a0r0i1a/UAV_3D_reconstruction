#include <ros/ros.h>
#include "motion_planning/GAEP/GAEPlanner.h"
#include <gflags/gflags.h>
#include <gperftools/profiler.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "GAEPlanner");
    
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, false);
    google::InstallFailureSignalHandler();

    // Start CPU profiler
    //ProfilerStart("/tmp/AEPlanner_profile_multi_frustum.out");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    GAEPlanner GAEPlanner(nh, nh_private);

    ros::spin();

    // Stop CPU profiler
    //ProfilerStop();
    //ROS_INFO("Profiler stopped. Raw data at /tmp/AEPlanner_profile_multi_frustum.out");

    // Optional: call system command to generate PDF automatically
    //std::string command = "google-pprof --pdf ~/motion_workspace/devel/lib/motion_planning/AEP /tmp/AEPlanner_profile_multi_frustum.out > /tmp/AEPlanner_profile_multi_frustum.pdf";
    //int ret = system(command.c_str());
    //if(ret == 0)
    //    ROS_INFO("Profiling PDF generated: /tmp/AEPlanner_profile_multi_frustum.pdf");
    //else
    //    ROS_WARN("Failed to generate profiling PDF. Run manually: %s", command.c_str());

    return 0;
}