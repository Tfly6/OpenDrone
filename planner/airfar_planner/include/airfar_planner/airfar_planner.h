#ifndef airfar_planner_H
#define airfar_planner_H

#include "utility.h"
#include "dynamic_graph.h"
#include "contour_detector.h"
#include "contour_graph.h"
#include "graph_planner.h"
#include "map_handler.h"
#include "planner_visualizer.h"
#include "scan_handler.h"
#include "graph_msger.h"


struct DPMasterParams {
    DPMasterParams() = default;
    float robot_dim; 
    float vehicle_height;
    float voxel_dim;
    float sensor_range;
    float waypoint_project_dist;
    std::string world_frame;
    float layer_resolution;
    int neighbor_layers;
    float main_run_freq;
    float viz_ratio;
    bool is_inter_navpoint;
    bool is_simulation;
    bool is_trajectory_edge;
    bool is_attempt_autoswitch;
};

class DPMaster {
public:
    DPMaster() = default;
    ~DPMaster() = default;

    void Init(); // Node initialization
    void Loop(); // Main Loop Function

private:
    ros::NodeHandle nh;
    ros::Subscriber reset_graph_sub_, joy_command_sub_;
    ros::Subscriber odom_sub_, terrain_sub_, terrian_local_sub_, scan_sub_, waypoint_sub_, target_sub_;
    ros::Publisher  goal_pub_;
    ros::Publisher  vertices_PCL_pub_, obs_world_pub_, new_PCL_pub_;
    ros::Publisher  dynamic_obs_pub_, surround_free_debug_, surround_obs_debug_, scan_grid_debug_, ground_pc_debug_;

    ros::Publisher mapping_time_pub_;

    Point3D robot_pos_, robot_heading_, nav_heading_, nav_goal_;

    bool is_robot_stop_, is_new_iter_, is_reset_env_;

    geometry_msgs::PointStamped goal_waypoint_stamped_;

    bool is_cloud_init_, is_scan_init_, is_odom_init_, is_planner_running_;
    bool is_goal_update_, is_dyobs_update_, is_graph_init_;

    std::vector<int> cur_layer_idxs_;

    PointCloudPtr new_vertices_ptr_;
    PointCloudPtr temp_obs_ptr_;
    PointCloudPtr temp_free_ptr_;
    PointCloudPtr temp_cloud_ptr_;
    PointCloudPtr local_terrian_ptr_;
    PointCloudPtr scan_grid_ptr_;

    NavNodePtr odom_node_ptr_ = NULL;
    NavNodePtr goal_node_ptr_ = NULL;
    NodePtrStack new_nodes_;
    NodePtrStack nav_graph_;
    NodePtrStack wide_nav_graph_;
    NodePtrStack clear_nodes_;

    CTNodeStack new_ctnodes_;

    tf::TransformListener* tf_listener_;

    /* module objects */
    ContourDetector contour_detector_;
    DynamicGraph graph_manager_;
    DPVisualizer planner_viz_;
    GraphPlanner graph_planner_;
    ContourGraph contour_graph_;
    MapHandler map_handler_;
    ScanHandler scan_handler_;
    GraphMsger graph_msger_;

    /* ROS Params */
    DPMasterParams      master_params_;
    ContourDetectParams cdetect_params_;
    DynamicGraphParams  graph_params_;
    GraphPlannerParams  gp_params_;
    ContourGraphParams  cg_params_;
    MapHandlerParams    map_params_;
    ScanHandlerParams   scan_params_;
    GraphMsgerParams    msger_parmas_;
    
    void LoadROSParams();

    void ResetEnvironmentAndGraph();
    void PrcocessCloud(const sensor_msgs::PointCloud2ConstPtr& pc,
                       const PointCloudPtr& cloudOut,
                       const bool& is_crop_cloud);

    Point3D ProjectNavWaypoint(const Point3D& nav_waypoint, const Point3D& last_waypoint);

    /* Callback Functions */
    void OdomCallBack(const nav_msgs::OdometryConstPtr& msg);
    void TerrainCallBack(const sensor_msgs::PointCloud2ConstPtr& pc);
    void TerrainLocalCallBack(const sensor_msgs::PointCloud2ConstPtr& pc);

    inline void ResetGraphCallBack(const std_msgs::EmptyConstPtr& msg) {
        is_reset_env_ = true;
    }

    inline void JoyCommandCallBack(const sensor_msgs::JoyConstPtr& msg) {
        if (msg->buttons[4] > 0.5) {
            is_reset_env_ = true;
        }
    }

    void ScanCallBack(const sensor_msgs::PointCloud2ConstPtr& pc);
    // void WaypointCallBack(const route_goal_msg::RouteGoal& routepoint);
    void WaypointCallBack(const geometry_msgs::PointStampedConstPtr & msg);
    void TargetCallBack(const geometry_msgs::PoseStampedConstPtr & msg);

    void ExtractDynamicObsFromScan(const PointCloudPtr& scanCloudIn, 
                                   const PointCloudPtr& obsCloudIn, 
                                   const PointCloudPtr& dyObsCloudOut);

    /* define inline functions */
    inline bool PreconditionCheck() {
        if (is_cloud_init_ && is_odom_init_) {
            return true;
        }
        return false;
    }
    inline void ClearTempMemory() {
        new_vertices_ptr_->clear();
        new_nodes_.clear();
        nav_graph_.clear();
        clear_nodes_.clear();
        new_ctnodes_.clear();
        wide_nav_graph_.clear();
    }

    inline void ResetInternalValues() {
        odom_node_ptr_ = NULL; 
        goal_node_ptr_ = NULL;
        is_cloud_init_      = false; 
        is_odom_init_       = false; 
        is_scan_init_       = false;
        is_planner_running_ = false; 
        is_goal_update_     = false;
        is_dyobs_update_    = false;  
        is_graph_init_      = false;
        is_robot_stop_      = false; 
        is_new_iter_        = false;
        ClearTempMemory();
    }
};

#endif