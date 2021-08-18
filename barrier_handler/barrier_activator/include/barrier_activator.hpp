#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_ros/point_cloud.h>
#include <numeric>
#include <dynamic_reconfigure/server.h>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <fstream>
#include "barrier_activator/BarrierActivatorConfig.h"
#include "std_msgs/Int16.h"


struct barrier
{
    int id;
    double x, y, z;
    double width, height, length;
};


class BarrierActivator
{
    public:
        BarrierActivator();

    private:
        ros::NodeHandle nh_, pnh_;
        ros::Subscriber sub, sub_selection, sub_save_current_box;
        ros::Publisher pub_id, pub_marker, pub_end_selection, pub_point, pub_marker_array;

        std::shared_ptr<dynamic_reconfigure::Server<barrier_activator::BarrierActivatorConfig> > dynamic_reconfigure_server_;

        void publishActiveBarrierCallback(int id);
        void configCallback(barrier_activator::BarrierActivatorConfig &config, uint32_t level);

        int active_barrier_id_, active_id;

        std_msgs::Int16 active_bar;
};



