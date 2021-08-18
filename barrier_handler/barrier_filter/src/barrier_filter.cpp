/*
BARRIER FILTER_____________________________________________________________________________________
This node should  be given a point of  interest (XYZ) in  /map frame so that it would crop the area 
around the POI  and perform euclidean  cluster extraction in /lidar/parent/os_sensor frome to check 
if an object  exists inside the area. Specifically, it  is detecting  whether a barrier  is in  its 
closed state or not. It then advertises a message  to notify anyone  interested. A subscriber  must
exist in order for it to advertise, otherwise the node will remain inactive.

-----------------------------

Created by: Onurcan Yucedag, 
            Can Ozcivelek
Date: 18/06/2021
*/

#include "barrier_filter.hpp"
#include "std_msgs/Bool.h"


// Class declaration.
BarrierFilter::BarrierFilter()
:   nh_(""),
    pnh_("~"),
    marker_duration(0.2),
    barrier_open_status(true),
    incoming_frame_count(0),
    min_number_points_(3),
    points_sub_shutdown(false)
{
    tf_buffer_ptr = std::make_shared<tf2_ros::Buffer>();
    tf_listener_ptr = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_ptr);

    // Create a ROS subscriber for the input point cloud.
    sub_points_ = nh_.subscribe("/lidar/parent/points_raw", 1, &BarrierFilter::PointsCallback, this);
    sub_active_barrier = pnh_.subscribe("/op_curr_stop_line_index", 1, &BarrierFilter::SetActiveBarrierCallback, this);

    // Advertise topics for processed cloud objects and visualization markers.
    pub_cropped_ = pnh_.advertise<sensor_msgs::PointCloud2> ("output_filtered", 1);
    pub_barrier_filter_markers_ = pnh_.advertise<visualization_msgs::MarkerArray>("barrier_vis", 1);
    pub_barrier_stat = pnh_.advertise<std_msgs::Bool>("is_barrier_safe", 1);

    barrier_history.resize(BARRIER_HISTORY_LENGTH);

    int total_num_of_barriers_;

    pnh_.getParam("/barrier_filter/total_num_barriers", total_num_of_barriers_);

    barrier_infos.resize(total_num_of_barriers_);

    for (int i = 0; i < total_num_of_barriers_; i++)
    {
        
        barrier_infos.at(i).id = i;
        // Get POI coordinate and dimension information from the parameter server.
        pnh_.getParam("barrier_" + std::to_string(i) + "/x", barrier_infos.at(i).x);
        pnh_.getParam("barrier_" + std::to_string(i) + "/y", barrier_infos.at(i).y);
        pnh_.getParam("barrier_" + std::to_string(i) + "/z", barrier_infos.at(i).z);
        pnh_.getParam("barrier_" + std::to_string(i) + "/width",  barrier_infos.at(i).width);
        pnh_.getParam("barrier_" + std::to_string(i) + "/height", barrier_infos.at(i).height);
        pnh_.getParam("barrier_" + std::to_string(i) + "/length", barrier_infos.at(i).length);
        pnh_.getParam("barrier_" + std::to_string(i) + "/stoplines",  barrier_infos.at(i).stoplines);

    }

    ROS_INFO_STREAM("* Barrier Filter Initialized Successfully...");

    initial_pose.pose.orientation.z = 0;
    initial_pose.pose.orientation.y = 0;
    initial_pose.pose.orientation.x = 0;
    initial_pose.pose.orientation.w = 1;


    cur_barrier_info.id = -999;
}

int BarrierFilter::GetBarrierFromStopLine(int stopline_id){
    for(const auto current_barrier : barrier_infos)
    {
        if(std::any_of(
                        std::begin(current_barrier.stoplines), std::end(current_barrier.stoplines),
                        [stopline_id](const auto & e) { return e == stopline_id; })){
            return current_barrier.id;
        }
    }

    return -1;
}


// ################################################################################################
// #### START - Callback to set currently activated barrier. ######################################
void BarrierFilter::SetActiveBarrierCallback(std_msgs::Int32 stopline_id)
{

    int barrier_id = GetBarrierFromStopLine(stopline_id.data);
    if(barrier_id == -1){
        if(!points_sub_shutdown){
            std::cout << "<< Shutted down the subscriber." << std::endl;
            points_sub_shutdown = true;
            sub_points_.shutdown();
        }
        return;
    }
    else {
        if(points_sub_shutdown){
            std::cout << "<< ReSubscribed to the points." << std::endl;
            points_sub_shutdown = false;
            sub_points_ = nh_.subscribe("/lidar/parent/points_raw", 1, &BarrierFilter::PointsCallback, this);
        }
        cur_barrier_info = barrier_infos.at(barrier_id);
    }
}
// #### END - Callback to set currently activated barrier. ########################################
// ################################################################################################


// ################################################################################################
// #### START - Callback to generate box around POI. ##############################################
void BarrierFilter::PublishVisualizationMarkers(geometry_msgs::PoseStamped & marker_pos, bool is_open, ros::Time marker_time)
{
    visualization_msgs::Marker barrier_box;
    std_msgs::ColorRGBA markers_color;

    if(is_open)
    {
        markers_color.r = 0.0;
        markers_color.g = 1.0;
        markers_color.b = 0.0;
        markers_color.a = 0.3;
    }  

    else
    {
        markers_color.r = 1.0;
        markers_color.g = 0.0;
        markers_color.b = 0.0;
        markers_color.a = 0.3;
    }

    // #### Define bounding box. ##################################################################
    // Marker for the lidar sensor calculated.
    barrier_box.header.frame_id = "map";
    barrier_box.header.stamp = ros::Time::now();
    barrier_box.ns = "barrier_box";
    barrier_box.id = 2;
    barrier_box.type = visualization_msgs::Marker::CUBE;
    barrier_box.action = visualization_msgs::Marker::MODIFY;

    barrier_box.pose.position.x = marker_pos.pose.position.x;
    barrier_box.pose.position.y = marker_pos.pose.position.y;
    barrier_box.pose.position.z = marker_pos.pose.position.z;

    barrier_box.pose.orientation.x = marker_pos.pose.orientation.x;
    barrier_box.pose.orientation.y = marker_pos.pose.orientation.y;
    barrier_box.pose.orientation.z = marker_pos.pose.orientation.z;
    barrier_box.pose.orientation.w = marker_pos.pose.orientation.w;

    barrier_box.scale.x = cur_barrier_info.width;
    barrier_box.scale.y = cur_barrier_info.length;
    barrier_box.scale.z = cur_barrier_info.height;

    barrier_box.color = markers_color;
    barrier_box.color.a = 0.3;
    
    barrier_box.lifetime = ros::Duration(marker_duration);
    barrier_box.header.stamp = marker_time;
    marker_arr.markers.push_back(barrier_box);
    // #### Define bounding box. ##################################################################


    // Status Text on Barrier
    visualization_msgs::Marker status_text;
    status_text.header.frame_id = "map";
    status_text.header.stamp = ros::Time::now();
    status_text.ns = "status_text";
    status_text.id = 3;
    status_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    status_text.action = visualization_msgs::Marker::MODIFY;

    status_text.pose.position.x = marker_pos.pose.position.x;
    status_text.pose.position.y = marker_pos.pose.position.y;
    status_text.pose.position.z = marker_pos.pose.position.z + 1.0;

    status_text.pose.orientation.x = marker_pos.pose.orientation.x;
    status_text.pose.orientation.y = marker_pos.pose.orientation.y;
    status_text.pose.orientation.z = marker_pos.pose.orientation.z;
    status_text.pose.orientation.w = marker_pos.pose.orientation.w;

    status_text.scale.z = 0.3;

    status_text.color = markers_color;
    status_text.color.a = 0.5;


    if(is_open)
        status_text.text = "OPEN";
    else
        status_text.text = "CLOSED";

    status_text.lifetime = ros::Duration(marker_duration);
    status_text.header.stamp = marker_time;

    marker_arr.markers.push_back(status_text);
    // Status Text on Barrier

    visualization_msgs::Marker indicator_clylinder;
    std_msgs::ColorRGBA cylinder_color;

    cylinder_color.r = 0.0;
    cylinder_color.g = 0.0;
    cylinder_color.b = 0.5;
    cylinder_color.a = 0.1;
      
    indicator_clylinder.header.frame_id = "map";
    indicator_clylinder.header.stamp = ros::Time::now();
    indicator_clylinder.ns = "barrier_box";
    indicator_clylinder.id = 4;
    indicator_clylinder.type = visualization_msgs::Marker::SPHERE;
    indicator_clylinder.action = visualization_msgs::Marker::MODIFY;

    indicator_clylinder.pose.position.x = marker_pos.pose.position.x;
    indicator_clylinder.pose.position.y = marker_pos.pose.position.y;
    indicator_clylinder.pose.position.z = marker_pos.pose.position.z;

    indicator_clylinder.pose.orientation.x = marker_pos.pose.orientation.x;
    indicator_clylinder.pose.orientation.y = marker_pos.pose.orientation.y;
    indicator_clylinder.pose.orientation.z = marker_pos.pose.orientation.z;
    indicator_clylinder.pose.orientation.w = marker_pos.pose.orientation.w;

    indicator_clylinder.scale.x = cur_barrier_info.width*8;
    indicator_clylinder.scale.y = cur_barrier_info.width*8;
    indicator_clylinder.scale.z = cur_barrier_info.width*8;

    indicator_clylinder.color = cylinder_color;
    indicator_clylinder.color.a = 0.4;
    
    indicator_clylinder.lifetime = ros::Duration(marker_duration);
    indicator_clylinder.header.stamp = marker_time;
    marker_arr.markers.push_back(indicator_clylinder);
    
    pub_barrier_filter_markers_.publish(marker_arr);
}
// #### END - Callback to generate box around POI. ################################################
// ################################################################################################


// ################################################################################################
// #### START - Callback to perform PCL clustering inside crop box. ###############################
void BarrierFilter::PointsCallback (const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
    // Check if the barrier stat topic is being asked by any other node.
    // Remain inactive if no subscribers.
    if(pub_barrier_stat.getNumSubscribers() < 1 && cur_barrier_info.id >= 0)
        return;

    // Construct and publish message
    std_msgs::Bool stat_msg;
    stat_msg.data = true;


    CloudT* cloud(new CloudT);
    CloudT::Ptr cloudPtr(cloud);
    CloudT::Ptr cloud_filtered(new CloudT);
    CloudT::Ptr cloud_f(new CloudT);

    geometry_msgs::PoseStamped transformed_pose;

    // Coordinates of any point of interest on the "map" frame.
    initial_pose.pose.position.x = cur_barrier_info.x;
    initial_pose.pose.position.y = cur_barrier_info.y;
    initial_pose.pose.position.z = cur_barrier_info.z;

    // Conversion to PCL format.
    pcl::fromROSMsg(*cloud_msg, *cloud);
    
    // Perform transformation from /map to /sensor frame.
    try
    {
        transformStamped = tf_buffer_ptr->lookupTransform(cloud_msg->header.frame_id, "map", ros::Time(0), ros::Duration(0.6));
        tf2::doTransform(initial_pose, transformed_pose, transformStamped);
    }

    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }

    // #### START - Cropbox filtering around the POI. #############################################
    pcl::CropBox<pcl::PointXYZ> crop_filter;
    crop_filter.setInputCloud(cloudPtr);

    // Pass the initial coordinates on map frame first.
    crop_filter.setMin(Eigen::Vector4f(initial_pose.pose.position.x - cur_barrier_info.width  / 2,
                                       initial_pose.pose.position.y - cur_barrier_info.length / 2,
                                       initial_pose.pose.position.z - cur_barrier_info.height / 2, 1.0));

    crop_filter.setMax(Eigen::Vector4f(initial_pose.pose.position.x + cur_barrier_info.width  / 2,
                                       initial_pose.pose.position.y + cur_barrier_info.length / 2,
                                       initial_pose.pose.position.z + cur_barrier_info.height / 2, 1.0));

    // Define a quaternion for the rotation.
    tf2::Quaternion q(transformStamped.transform.rotation.x,
                      transformStamped.transform.rotation.y,
                      transformStamped.transform.rotation.z,
                      transformStamped.transform.rotation.w);

    // Get RPY values using quaternion.
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // Rotate and translate the initial point to transform it to sensor frame.
    crop_filter.setRotation((Eigen::Vector3f() << roll, pitch, yaw).finished());
    crop_filter.setTranslation((Eigen::Vector3f() << transformStamped.transform.translation.x,
                                                     transformStamped.transform.translation.y,
                                                     transformStamped.transform.translation.z).finished());
    // Apply crop filter.
    crop_filter.filter(*cloud_filtered);
    // #### END - Cropbox filtering around the POI. ###############################################

    // conversion back to ROS format.
    pcl::toROSMsg(*cloud_filtered, output);

    output.header = cloud_msg->header;

    // Assign data to be published to their respective publishers.
    pub_cropped_.publish(output);

    // Print message of barrier detected when there are more than 10 points detected inside cropbox.
    if(cloud_filtered->points.size() < min_number_points_)
    {
        pub_barrier_stat.publish(stat_msg);

        barrier_history.at(incoming_frame_count%BARRIER_HISTORY_LENGTH) = stat_msg.data;
        const float total_history = std::accumulate(barrier_history.begin(), barrier_history.end(), 0.0);

        barrier_open_status = total_history/BARRIER_HISTORY_LENGTH > OPEN_TO_CLOSED_RATE;
        PublishVisualizationMarkers(initial_pose, barrier_open_status, cloud_msg->header.stamp);
        ++incoming_frame_count;
        return;
    }

    // #### Extraction of clusters ################################################################
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);

    tree->setInputCloud(cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    
    ec.setClusterTolerance(0.3);
    ec.setMinClusterSize(min_number_points_);
    ec.setMaxClusterSize(10000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered);
    ec.extract(cluster_indices);
    
    // #### Extraction of clusters ################################################################

    // Confirm barrier if there at least one cluster.
    stat_msg.data = cluster_indices.empty();

    pub_barrier_stat.publish(stat_msg);

    barrier_history.at(incoming_frame_count%BARRIER_HISTORY_LENGTH) = stat_msg.data;
    const float total_history = std::accumulate(barrier_history.begin(), barrier_history.end(), 0.0);
    barrier_open_status = total_history / BARRIER_HISTORY_LENGTH > OPEN_TO_CLOSED_RATE;

    PublishVisualizationMarkers(initial_pose, barrier_open_status, cloud_msg->header.stamp);
    ++incoming_frame_count;

}
// #### END - Callback to perform PCL clustering inside crop box. #################################
// ################################################################################################
