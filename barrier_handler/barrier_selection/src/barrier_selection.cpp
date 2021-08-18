#include "ros/ros.h"
#include "barrier_selection.hpp"

int main(int argc, char** argv)
{
    // Initialize ROS.barrier_selection
    ros::init (argc, argv, "barrier_selection");

    BarrierSelection barrier_selection;
    // Spin
    ros::spin ();
}

