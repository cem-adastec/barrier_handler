#include "barrier_activator.hpp"


BarrierActivator::BarrierActivator() : 
nh_(""),
pnh_("~")
{
    pnh_.param<int>("barrier_id", active_barrier_id_, 1);

    pub_id = pnh_.advertise<std_msgs::Int16>("barrier_id", 1);

    dynamic_reconfigure_server_ = std::make_shared<dynamic_reconfigure::Server<barrier_activator::BarrierActivatorConfig> >();
    dynamic_reconfigure::Server<barrier_activator::BarrierActivatorConfig>::CallbackType f;
    f = boost::bind(&BarrierActivator::configCallback, this, _1, _2);
    dynamic_reconfigure_server_->setCallback(f);
}


// ################################################################################################
// #### START - Callback to publish currently activated barrier id. ###############################
void BarrierActivator::publishActiveBarrierCallback(int id)
{
    active_bar.data = id;
    pub_id.publish(active_bar);
}
// #### END - Callback to publish currently activated barrier id. #################################
// ################################################################################################


// ################################################################################################
// #### START - Callback to listen to reconfigure for dynamic parameter changes. ##################
void BarrierActivator::configCallback(barrier_activator::BarrierActivatorConfig &config, uint32_t level)
{
    active_barrier_id_ = config.active_barrier_id;
    // Update parameters for dynamically changing parameters.
    std::cout << "Active barrier has been changed: " << std::endl;
    std::cout << "\t* Active barrier id: " << active_barrier_id_ << std::endl;
    std::cout << "---------------------------------------------"<< std::endl;

    publishActiveBarrierCallback(active_barrier_id_);
}
// #### END - Callback to listen to reconfigure for dynamic parameter changes. ####################
// ################################################################################################

