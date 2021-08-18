/*
BARRIER SELECTION__________________________________________________________________________________
This node helps creating points of interest on a PCD. By publishing points on RViz,one can generate 
a bounding box around a POI and save them as a config file. Specifically, this node was created for 
selecting barriers on maps to later detect them using the barrier_filter node and publish its state.

-----------------------------

Created by: Can Ozcivelek
Date: 24/06/2021
*/

#include "barrier_selection.hpp"


BarrierSelection::BarrierSelection() : 
nh_(""), 
pnh_("~"), 
marker_duration(0.2),
min_number_points_(3),
box_x(4.5),
box_y(1.0),
box_z(1.0)
{
    // Create a ROS subscriber for the input point cloud.
    sub_clicked_pt = nh_.subscribe("clicked_point", 1, &BarrierSelection::clickedPtCallback, this);
    sub_selection = nh_.subscribe("generate_box", 1, &BarrierSelection::generateMarkerCallback, this);
    sub_save_current_box = nh_.subscribe("save_current_box", 1, &BarrierSelection::saveCurrentBoxCallback, this);
    sub_reset_current_box = nh_.subscribe("reset_current_box", 1, &BarrierSelection::resetCurrentBoxCallback, this);
    sub_stoplines = nh_.subscribe("vector_map", 1, &BarrierSelection::getStopLinePoints, this);
    // Advertise topics for processed cloud objects.
    pub = pnh_.advertise<geometry_msgs::PointStamped> ("selected_point", 1);
    pub_marker = pnh_.advertise<visualization_msgs::MarkerArray> ("selected_mark", 1);
    pub_marker_array = pnh_.advertise<visualization_msgs::MarkerArray> ("all_selected_pts", 1);

    std::cout << "* Barrier Selection Initialized Successfully..." << std::endl;
    std::cout << "\n\t________Instructions________\n" 
              << "\t1. Publish points around the point of interest\n"
              << "\t2. Generate box to visualize bounding box\n"
              << "\t3. If not happy with the box placement/dimensions, bring up rqt_reconfigure to modify\n"
              << "\t4. When done, save current box params inside a yaml file\n"
              << "\t5. Repeat for each barrier.\n\n";


    // Rqt Dynamic Reconfigure and Parameter Definitions
    pnh_.param<double>("barrier_center_x_adjust", barrier_box_x_, 0.0);
    pnh_.param<double>("barrier_center_y_adjust", barrier_box_y_, 0.0);
    pnh_.param<double>("barrier_center_z_adjust", barrier_box_z_, 0.0);

    pnh_.param<double>("barrier_box_width_adjust" , barrier_box_width_ , 0.0);
    pnh_.param<double>("barrier_box_height_adjust", barrier_box_height_, 0.0);
    pnh_.param<double>("barrier_box_length_adjust", barrier_box_length_, 0.0);

    pnh_.getParam("path_to_save", path_to_save_);

    dynamic_reconfigure_server_ = std::make_shared<dynamic_reconfigure::Server<barrier_selection::BarrierSelectionConfig> >();
    dynamic_reconfigure::Server<barrier_selection::BarrierSelectionConfig>::CallbackType f;
    f = boost::bind(&BarrierSelection::configCallback, this, _1, _2);
    dynamic_reconfigure_server_->setCallback(f);
}


// ################################################################################################
// #### START - Callback to receive clicked point info and calculate bounding box coords & dims. ##
void BarrierSelection::clickedPtCallback(geometry_msgs::PointStamped clicked_pt_msg)
{
    click_count++;
    box_generated = false;
    barrier_points.push_back(clicked_pt_msg);

    // #### START - Find min, max & avg values from selected points ###############################
    // This will be used to find centroid of the selected points,
    // As well as help in calculating bbox dimensions.
    max_x = barrier_points[0].point.x;
    max_y = barrier_points[0].point.y;
    max_z = barrier_points[0].point.z;
    min_x = barrier_points[0].point.x;
    min_y = barrier_points[0].point.y;
    min_z = barrier_points[0].point.z;

    for(int i = 0; i < barrier_points.size(); i++)
    {
        if(barrier_points[i].point.x > max_x)
            max_x = barrier_points[i].point.x;

        if(barrier_points[i].point.y > max_y)
            max_y = barrier_points[i].point.y;

        if(barrier_points[i].point.z > max_z)
            max_z = barrier_points[i].point.z;

        if(barrier_points[i].point.x < min_x)
            min_x = barrier_points[i].point.x;

        if(barrier_points[i].point.y < min_y)
            min_y = barrier_points[i].point.y;

        if(barrier_points[i].point.z < min_z)
            min_z = barrier_points[i].point.z;
    }

    avg_x = (min_x + max_x) / 2;
    avg_y = (min_y + max_y) / 2;
    avg_z = (min_z + max_z) / 2;

    box_x = max_x - min_x;
    box_y = max_y - min_y;
    box_z = max_z - min_z;
    // #### END - Find min, max & avg values from selected points #################################

    // #### START - Visualize all selected points as spheres ######################################
    for(int i = 0; i < barrier_points.size(); i++)
    {
        selected_point_vis.header.frame_id = "/map";
        selected_point_vis.header.stamp = ros::Time::now();
        selected_point_vis.ns = "basic_shapes";
        selected_point_vis.id = i;
        selected_point_vis.type = visualization_msgs::Marker::SPHERE;
        selected_point_vis.action = visualization_msgs::Marker::ADD;
        selected_point_vis.pose.position.x = barrier_points[i].point.x;
        selected_point_vis.pose.position.y = barrier_points[i].point.y;
        selected_point_vis.pose.position.z = barrier_points[i].point.z;
        selected_point_vis.pose.orientation.x = 0.0;
        selected_point_vis.pose.orientation.y = 0.0;
        selected_point_vis.pose.orientation.z = 0.0;
        selected_point_vis.pose.orientation.w = 0.0;
        selected_point_vis.scale.x = 0.1;
        selected_point_vis.scale.y = 0.1;
        selected_point_vis.scale.z = 0.1;
        selected_point_vis.color.r = 1.0;
        selected_point_vis.color.g = 0.0;
        selected_point_vis.color.b = 0.0;
        selected_point_vis.color.a = 0.5;
        selected_point_vis.lifetime = ros::Duration();
        all_selected_points.markers.push_back(selected_point_vis);
    }
    // #### END - Visualize all selected points as spheres ########################################

    ROS_WARN_STREAM("Number of points for the current barrier: " << all_selected_points.markers.size());
    pub_marker_array.publish(all_selected_points);
    all_selected_points.markers.clear();
}
// #### END - Callback to receive clicked point info and calculate bounding box coords & dims. ####
// ################################################################################################


// ################################################################################################
// #### START - Callback to visualize bbox and save coordinate and dimension information. #########
void BarrierSelection::generateMarkerCallback(const std_msgs::Empty& input)
{       
    box_generated = true;

    // #### START - Generating crop box around the selected points ################################
    barrier_box.header.frame_id = "/map";
    barrier_box.header.stamp = ros::Time::now();
    barrier_box.ns = "basic_shapes";
    barrier_box.id = 2;
    barrier_box.type = visualization_msgs::Marker::CUBE;
    barrier_box.action = visualization_msgs::Marker::ADD;
    barrier_box.pose.position.x = avg_x + barrier_box_x_;
    barrier_box.pose.position.y = avg_y + barrier_box_y_;
    barrier_box.pose.position.z = avg_z + barrier_box_z_;
    barrier_box.pose.orientation.x = 0.0;
    barrier_box.pose.orientation.y = 0.0;
    barrier_box.pose.orientation.z = 0.0;
    barrier_box.pose.orientation.w = 0.0;
    barrier_box.scale.x = box_x + barrier_box_width_  + box_margin;
    barrier_box.scale.y = box_y + barrier_box_length_ + box_margin;
    barrier_box.scale.z = box_z + barrier_box_height_ + box_margin;
    barrier_box.color.r = 0.0f;
    barrier_box.color.g = 1.0f;
    barrier_box.color.b = 0.0f;
    barrier_box.color.a = 0.3;
    barrier_box.lifetime = ros::Duration();
    markers_pub.markers.push_back(barrier_box);
    // #### END - Generating crop box around the selected points ##################################

    std_msgs::ColorRGBA cylinder_color;

    cylinder_color.r = 0.0;
    cylinder_color.g = 0.0;
    cylinder_color.b = 0.5;
    cylinder_color.a = 0.1;
      
    indicator_clylinder.header.frame_id = "/map";
    indicator_clylinder.header.stamp = ros::Time::now();
    indicator_clylinder.ns = "barrier_sphere";
    indicator_clylinder.id = 4;
    indicator_clylinder.type = visualization_msgs::Marker::SPHERE;
    indicator_clylinder.action = visualization_msgs::Marker::ADD;

    indicator_clylinder.pose.position.x = barrier_box.pose.position.x;
    indicator_clylinder.pose.position.y = barrier_box.pose.position.y;
    indicator_clylinder.pose.position.z = barrier_box.pose.position.z;

    indicator_clylinder.pose.orientation.x = barrier_box.pose.orientation.x;
    indicator_clylinder.pose.orientation.y = barrier_box.pose.orientation.y;
    indicator_clylinder.pose.orientation.z = barrier_box.pose.orientation.z;
    indicator_clylinder.pose.orientation.w = barrier_box.pose.orientation.w;

    indicator_clylinder.scale.x = barrier_stopline_radius;
    indicator_clylinder.scale.y = barrier_stopline_radius;
    indicator_clylinder.scale.z = barrier_stopline_radius;

    indicator_clylinder.color = cylinder_color;
    indicator_clylinder.lifetime = ros::Duration();
    markers_pub.markers.push_back(indicator_clylinder);

    pub_marker.publish(markers_pub);
    calculateDistances();
    // Fill in the vectors with corrdinate and scale values
    all_barrier_points.push_back(barrier_points);
    xyz.push_back(barrier_box.pose.position.x);
    xyz.push_back(barrier_box.pose.position.y);
    xyz.push_back(barrier_box.pose.position.z);
    wlh.push_back(barrier_box.scale.x);
    wlh.push_back(barrier_box.scale.y);
    wlh.push_back(barrier_box.scale.z);
    all_xyz.push_back(xyz);
    all_wlh.push_back(wlh);
    are_stopline_instersect_all_barrier.push_back(are_stopline_intersect);
    barrier_points.clear();
    xyz.clear();
    wlh.clear();
    are_stopline_intersect.clear();
    click_count = 0;
}
// #### END - Callback to visualize bbox and save coordinate and dimension information. ###########
// ################################################################################################


// ################################################################################################
// #### START - Callback to listen to reconfigure for dynamic parameter changes. ##################
void BarrierSelection::configCallback(barrier_selection::BarrierSelectionConfig &config, uint32_t level)
{
    // Update parameters for dynamically changing parameters.
    std::cout << "Parameters have been changed: " << std::endl;
    std::cout << "\t* new barrier_center_x: " << barrier_box.pose.position.x << std::endl;
    std::cout << "\t* new barrier_center_y: " << barrier_box.pose.position.y << std::endl;
    std::cout << "\t* new barrier_center_z: " << barrier_box.pose.position.z << std::endl;
    std::cout << "\t* new barrier_box_width: " <<  barrier_box.scale.x << std::endl;
    std::cout << "\t* new barrier_box_height: " << barrier_box.scale.y << std::endl;
    std::cout << "\t* new barrier_box_length: " << barrier_box.scale.z << std::endl;
    std::cout << "---------------------------------------------"<< std::endl;
    markers_pub.markers.clear();
    barrier_box_x_ = config.barrier_box_x_adjust;
    barrier_box_y_ = config.barrier_box_y_adjust;
    barrier_box_z_ = config.barrier_box_z_adjust;
    barrier_box_width_ = config.barrier_box_width_adjust;
    barrier_box_height_ = config.barrier_box_height_adjust;
    barrier_box_length_ = config.barrier_box_length_adjust;
    barrier_stopline_radius = config.barrier_stopline_matcher_radius;
    barrier_box.header.frame_id = "/map";
    barrier_box.header.stamp = ros::Time::now();
    barrier_box.ns = "basic_shapes";
    barrier_box.id = 2;
    barrier_box.type = visualization_msgs::Marker::CUBE;
    barrier_box.action = visualization_msgs::Marker::ADD;
    barrier_box.pose.position.x = avg_x + barrier_box_x_;
    barrier_box.pose.position.y = avg_y + barrier_box_y_;
    barrier_box.pose.position.z = avg_z + barrier_box_z_;
    barrier_box.pose.orientation.x = 0.0;
    barrier_box.pose.orientation.y = 0.0;
    barrier_box.pose.orientation.z = 0.0;
    barrier_box.pose.orientation.w = 0.0;
    barrier_box.scale.x = box_x + box_margin + barrier_box_width_;
    barrier_box.scale.y = box_y + box_margin + barrier_box_length_;
    barrier_box.scale.z = box_z + box_margin + barrier_box_height_;
    barrier_box.color.r = 0.0f;
    barrier_box.color.g = 1.0f;
    barrier_box.color.b = 0.0f;
    barrier_box.color.a = 0.3;
    barrier_box.lifetime = ros::Duration();

    markers_pub.markers.push_back(barrier_box);

    std_msgs::ColorRGBA cylinder_color;

    cylinder_color.r = 0.0;
    cylinder_color.g = 0.0;
    cylinder_color.b = 0.5;
    cylinder_color.a = 0.3;
      
    indicator_clylinder.header.frame_id = "/map";
    indicator_clylinder.header.stamp = ros::Time::now();
    indicator_clylinder.ns = "barrier_sphere";
    indicator_clylinder.id = 4;
    indicator_clylinder.type = visualization_msgs::Marker::SPHERE;
    indicator_clylinder.action = visualization_msgs::Marker::ADD;

    indicator_clylinder.pose.position.x = barrier_box.pose.position.x;
    indicator_clylinder.pose.position.y = barrier_box.pose.position.y;
    indicator_clylinder.pose.position.z = barrier_box.pose.position.z;

    indicator_clylinder.pose.orientation.x = barrier_box.pose.orientation.x;
    indicator_clylinder.pose.orientation.y = barrier_box.pose.orientation.y;
    indicator_clylinder.pose.orientation.z = barrier_box.pose.orientation.z;
    indicator_clylinder.pose.orientation.w = barrier_box.pose.orientation.w;

    indicator_clylinder.scale.x = barrier_stopline_radius*2;
    indicator_clylinder.scale.y = barrier_stopline_radius*2;
    indicator_clylinder.scale.z = barrier_stopline_radius*2;

    indicator_clylinder.color = cylinder_color;
    indicator_clylinder.lifetime = ros::Duration();
    markers_pub.markers.push_back(indicator_clylinder);



    
    pub_marker.publish(markers_pub);
    calculateDistances();
    // Fill in the vectors with corrdinate and scale values
    // Only if generate box button is pressed.
    if(box_generated)
    {
        all_xyz.pop_back();
        all_wlh.pop_back();
        are_stopline_instersect_all_barrier.pop_back();
        xyz.push_back(barrier_box.pose.position.x);
        xyz.push_back(barrier_box.pose.position.y);
        xyz.push_back(barrier_box.pose.position.z);
        wlh.push_back(barrier_box.scale.x);
        wlh.push_back(barrier_box.scale.y);
        wlh.push_back(barrier_box.scale.z);
        all_xyz.push_back(xyz);
        all_wlh.push_back(wlh);
        are_stopline_instersect_all_barrier.push_back(are_stopline_intersect);
        
    }

    barrier_points.clear();
    xyz.clear();
    wlh.clear();
    are_stopline_intersect.clear();
    click_count = 0;
}
// #### END - Callback to listen to reconfigure for dynamic parameter changes. ####################
// ################################################################################################


// ################################################################################################
// #### START - Callback to reset bounding box. ###################################################
void BarrierSelection::resetCurrentBoxCallback(const std_msgs::Empty& input)
{
    // Terminal output for better user interaction.
    ROS_WARN_STREAM("resetCurrentBoxCallback");

    selected_point_vis.action = visualization_msgs::Marker::DELETEALL;
    all_selected_points.markers.push_back(selected_point_vis);

    if(box_generated)
    {
        
        indicator_clylinder.action = visualization_msgs::Marker::DELETEALL;
        selected_point_vis.action = visualization_msgs::Marker::DELETEALL;
        barrier_box.action = visualization_msgs::Marker::DELETEALL;
        all_selected_points.markers.push_back(selected_point_vis);
        markers_pub.markers.push_back(indicator_clylinder);
        markers_pub.markers.push_back(barrier_box);
        // xyz.pop_back();
        // wlh.pop_back();
        all_xyz.pop_back();
        all_wlh.pop_back();
        are_stopline_instersect_all_barrier.pop_back();
        all_barrier_points.pop_back();
    }
    
    pub_marker_array.publish(all_selected_points);
    pub_marker.publish(markers_pub);
    all_selected_points.markers.clear();
    markers_pub.markers.clear();
    barrier_points.clear();
    //are_stopline_intersect.clear();
    
    
}
// #### END - Callback to reset bounding box. #####################################################
// ################################################################################################


// ################################################################################################
// #### START - Callback to save parameters inside a config file. #################################
void BarrierSelection::saveCurrentBoxCallback(const std_msgs::Empty& input)
{
    // Terminal output for better user interaction.
    ROS_INFO_STREAM("Saved!");
    ROS_INFO_STREAM("File at: " << path_to_save_ << file_name);
    ROS_INFO_STREAM("Number of constructed barriers: " << all_barrier_points.size());
    ROS_INFO_STREAM("To construct another barrier, repeat process from publishing points.");
    ROS_INFO_STREAM("Ctrl+C to end process.");

    box_generated = false;


    // Save config file to the defined path with a specific structure.
    std::ofstream my_file(path_to_save_ + file_name);
    if(my_file.is_open())
    {
        for(int i = 0; i < all_xyz.size(); i++)
        {
            my_file << "barrier_" << i + 1 << ":\n";
            my_file << "  " << "id: " << i + 1 << "\n";
            my_file << "  " << "stoplines: " << getStopLineStr(are_stopline_instersect_all_barrier.at(i)) << "\n";
            my_file << "  " << "x: " << all_xyz[i][0] << "\n";
            my_file << "  " << "y: " << all_xyz[i][1] << "\n";
            my_file << "  " << "z: " << all_xyz[i][2] << "\n";
            my_file << "  " << "width: "  << all_wlh[i][0] << "\n";
            my_file << "  " << "length: " << all_wlh[i][1] << "\n";
            my_file << "  " << "height: " << all_wlh[i][2] << "\n\n";
        }
        my_file << "total_num_barriers: " << all_barrier_points.size();
        my_file.close();
        ROS_INFO_STREAM("SELAMI.");
    }

    // File not found.
    else 
    {   
        ROS_WARN_STREAM("Could not open file!");
        return;
    }
}
// #### END - Callback to save parameters inside a config file. ###################################
// ################################################################################################

void BarrierSelection::getStopLinePoints(visualization_msgs::MarkerArray stopline_info)
{
    std::vector<visualization_msgs::Marker> markers = stopline_info.markers;
    stopline_points.resize(markers.size());

    for (size_t i = 0; i < markers.size(); i++)
    {
        
       stopline_points.at(i) = markers.at(i).points;
    }
    
}


void BarrierSelection::calculateDistances(){
    are_stopline_intersect.resize(stopline_points.size());
    std::vector<double> sphere_center = {indicator_clylinder.pose.position.x,
                                         indicator_clylinder.pose.position.y,
                                         indicator_clylinder.pose.position.z};
    
    for (size_t i = 0; i < stopline_points.size(); i++)
    {
        std::vector<double> p1p2 = {
            stopline_points.at(i).at(1).x - stopline_points.at(i).at(0).x,
            stopline_points.at(i).at(1).y - stopline_points.at(i).at(0).y,
            stopline_points.at(i).at(1).z - stopline_points.at(i).at(0).z
        };
        std::vector<double> p1p0 = {
            sphere_center.at(0) - stopline_points.at(i).at(0).x,
            sphere_center.at(1) - stopline_points.at(i).at(0).y,
            sphere_center.at(2) - stopline_points.at(i).at(0).z
        };
        
        double t = dot_product(p1p0,p1p2)/squared_length(p1p2);
        std::vector<double> closest_point(3);
        if (t<0)
        {
            closest_point.at(0) = stopline_points.at(i).at(0).x;
            closest_point.at(1) = stopline_points.at(i).at(0).y;
            closest_point.at(2) = stopline_points.at(i).at(0).z;
        }else if (t>1)
        {
            closest_point.at(0) = stopline_points.at(i).at(1).x;
            closest_point.at(1) = stopline_points.at(i).at(1).y;
            closest_point.at(2) = stopline_points.at(i).at(1).z;
        }else{
            closest_point.at(0) = stopline_points.at(i).at(0).x + t*p1p2.at(0);
            closest_point.at(1) = stopline_points.at(i).at(0).y + t*p1p2.at(1);
            closest_point.at(2) = stopline_points.at(i).at(0).z + t*p1p2.at(2);

        }
        if (squared_length(distance_vector(closest_point, sphere_center)) <= barrier_stopline_radius*barrier_stopline_radius)
        {
            are_stopline_intersect.at(i) = true;
        }else{
            are_stopline_intersect.at(i) = false;
        }
        std::cout<< are_stopline_intersect.at(i) << std::endl;
        
    }
    
}

float BarrierSelection::dot_product(std::vector<double> vector_a, std::vector<double> vector_b) {
   float product = 0;
   for (int i = 0; i < vector_a.size(); i++)
   product = product + vector_a.at(i) * vector_b.at(i);
   return product;
}
float BarrierSelection::squared_length(std::vector<double> vector_a){
    float sum = 0;
    for (size_t i = 0; i < vector_a.size(); i++)
    {
      sum += vector_a.at(i)*vector_a.at(i);
    }
    return sum;
}

std::vector<double> BarrierSelection::distance_vector(std::vector<double> vector_a, std::vector<double> vector_b){
    std::vector<double> vector_res = {vector_a.at(0) - vector_b.at(0),
                                      vector_a.at(1) - vector_b.at(1),
                                      vector_a.at(2) - vector_b.at(2)
                                            
    };
    return vector_res;
}

std::string BarrierSelection::getStopLineStr(std::vector<bool> arr){
    std::string result = "[";
    if (arr.at(0) == true)
    {
       result.append("1");
    }
    
    
    for (size_t i = 1; i < arr.size(); i++)
    {
        if (arr.at(i) == true)
        {
            result.append(" ,");
            result.append("%d", i+1);
        }
        
    }
    result.append("]");
    return result;
}