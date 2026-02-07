#include <ros/ros.h>
#include <chrono> // For timing

extern "C" void launch_aep_kernel();

int main(int argc, char** argv) {
    ros::init(argc, argv, "gpu_sanity_check");
    ros::NodeHandle nh;

    ROS_INFO("--- 1. Cold Start (Initializing Driver) ---");
    auto start = std::chrono::high_resolution_clock::now();
    
    launch_aep_kernel(); // This will take ~0.8s
    
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> diff = end - start;
    ROS_INFO("First Run Time: %f s", diff.count());

    ROS_INFO("--- 2. Warm Run (Actual Speed) ---");
    start = std::chrono::high_resolution_clock::now();
    
    launch_aep_kernel(); // This should be nearly instant
    
    end = std::chrono::high_resolution_clock::now();
    diff = end - start;
    ROS_INFO("Second Run Time: %f s", diff.count());

    return 0;
}