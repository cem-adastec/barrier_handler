#include "ros/ros.h"
#include "barrier_activator.hpp"

int main(int argc, char** argv)
{
    // Initialize ROS.barrier_selection
    ros::init (argc, argv, "barrier_activator");

    BarrierActivator barrier_activator;
    // Spin
    ros::spin ();
}

