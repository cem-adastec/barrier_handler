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
#include "std_msgs/Int32.h"


typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> CloudT;

#define BARRIER_HISTORY_LENGTH 15
#define OPEN_TO_CLOSED_RATE 0.7

struct BarrierInfo{
    int id;
    float x;
    float y;
    float z;
    float width;
    float length;
    float height;
    std::vector<int> stoplines;
};

class BarrierFilter
{    
    public:
        BarrierFilter();


    private:
        ros::NodeHandle nh_, pnh_;

        ros::Publisher pub_cropped_,
                       pub_barrier_filter_markers_,
                       pub_barrier_stat;

        ros::Subscriber sub_points_, sub_active_barrier;

        // Define output cloud.
        sensor_msgs::PointCloud2 output;
        sensor_msgs::PointCloud2 ec_output;

        sensor_msgs::PointCloud2::Ptr clusters;

        std::shared_ptr<tf2_ros::Buffer> tf_buffer_ptr;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_ptr;

        geometry_msgs::TransformStamped transformStamped;
        geometry_msgs::PoseStamped initial_pose;

        visualization_msgs::MarkerArray marker_arr;

        float marker_duration;

        std::vector<BarrierInfo> barrier_infos;
        BarrierInfo cur_barrier_info;

        std::vector<bool> barrier_history;
        bool barrier_open_status;
        int incoming_frame_count;

        std::vector<std::string> bars;

        int min_number_points_;
        int total_num_of_barriers_;
        bool points_sub_shutdown;

        void PointsCallback (const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);
        void PublishVisualizationMarkers(geometry_msgs::PoseStamped& marker_pos, bool is_open, ros::Time marker_time);
        void SetActiveBarrierCallback(std_msgs::Int32 stopline_id);
        int GetBarrierFromStopLine(int stopline_id);

};

