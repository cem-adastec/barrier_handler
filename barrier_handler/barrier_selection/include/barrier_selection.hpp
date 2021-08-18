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
#include "barrier_selection/BarrierSelectionConfig.h"

struct barrier
{
    int id;
    double x, y, z;
    double width, height, length;
};


class BarrierSelection
{
    public:
        BarrierSelection();

    private:
        ros::NodeHandle nh_, pnh_;
        ros::Subscriber sub_clicked_pt, sub_selection, sub_save_current_box, sub_reset_current_box, sub_stoplines;
        ros::Publisher pub, pub_marker, pub_end_selection, pub_point, pub_marker_array;

        std::shared_ptr<dynamic_reconfigure::Server<barrier_selection::BarrierSelectionConfig> > dynamic_reconfigure_server_;

        void clickedPtCallback(geometry_msgs::PointStamped clicked_pt_msg);
        void generateMarkerCallback(const std_msgs::Empty& input);
        void configCallback(barrier_selection::BarrierSelectionConfig &config, uint32_t level);
        void saveCurrentBoxCallback(const std_msgs::Empty& input);
        void resetCurrentBoxCallback(const std_msgs::Empty& input);
        void getStopLinePoints(visualization_msgs::MarkerArray stopline_info);
        void calculateDistances();
        float dot_product(std::vector<double> vector_a, std::vector<double> vector_b);
        float squared_length(std::vector<double> vector_a);
        std::vector<double> distance_vector(std::vector<double> vector_a, std::vector<double> vector_b);
        std::string getStopLineStr(std::vector<bool> arr);

        std::vector<geometry_msgs::PointStamped> barrier_points;
        std::vector<std::vector<geometry_msgs::PointStamped>> all_barrier_points;

        visualization_msgs::Marker barrier_box, selected_point_vis, indicator_clylinder;
        visualization_msgs::MarkerArray all_markers, all_selected_points, markers_pub;

        std::string path_to_save_, file_name = "barrier_config.yaml";

        std::vector<double> wlh, xyz;
        std::vector<std::vector<double>> all_wlh, all_xyz;
        std::vector<bool> are_stopline_intersect;
        std::vector<std::vector<bool>> are_stopline_instersect_all_barrier;
        std::vector<std::vector<geometry_msgs::Point>> stopline_points;
        bool box_generated = false;

        int click_count = 0,
            min_number_points_;

        double marker_duration = 0.2,
               avg_x, avg_y, avg_z, 
               min_x, min_y, min_z, 
               max_x, max_y, max_z,
               box_x, box_y, box_z,
               box_margin = 0.3;
            
        double barrier_box_x_ = 0,
               barrier_box_y_ = 0,
               barrier_box_z_ = 0,
               barrier_box_width_ = 0,
               barrier_box_height_ = 0,
               barrier_box_length_ = 0,
               barrier_stopline_radius = 0;
};



