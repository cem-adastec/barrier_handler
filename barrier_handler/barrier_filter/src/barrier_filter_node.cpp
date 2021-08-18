#include "ros/ros.h"
#include "barrier_filter.hpp"

int main(int argc, char** argv)
{
    // Initialize ROS.
    ros::init (argc, argv, "barrier_filter");

    BarrierFilter barrier_filter;
    
    // Spin
    ros::spin ();
}

