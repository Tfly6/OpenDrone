/*
 * Dynamic Route Planner
 * Copyright (C) 2021 Fan Yang - All rights reserved
 * fanyang2@andrew.cmu.edu,   
 */



#include "airfar_planner/airfar_planner.h"

/***************************************************************************************/

void DPMaster::Init() {
  /* initialize subscriber and publisher */
  reset_graph_sub_   = nh.subscribe("/reset_visibility_graph", 5, &DPMaster::ResetGraphCallBack, this);
  odom_sub_          = nh.subscribe("/odom_world", 5, &DPMaster::OdomCallBack, this);
  terrain_sub_       = nh.subscribe("/terrain_cloud", 1, &DPMaster::TerrainCallBack, this);
  scan_sub_          = nh.subscribe("/scan_cloud", 5, &DPMaster::ScanCallBack, this);
  waypoint_sub_      = nh.subscribe("/goal", 1, &DPMaster::WaypointCallBack, this);
  target_sub_        = nh.subscribe("/move_base_simple/goal", 1, &DPMaster::TargetCallBack, this);
  terrian_local_sub_ = nh.subscribe("/terrain_local_cloud", 1, &DPMaster::TerrainLocalCallBack, this);
  joy_command_sub_   = nh.subscribe("/joy", 5, &DPMaster::JoyCommandCallBack, this);

  goal_pub_ = nh.advertise<geometry_msgs::PointStamped>("/way_point",5);

  vertices_PCL_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/vertics",5);
  obs_world_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/dynamic_obs_world",1);
  new_PCL_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/new_PCL",1);
  mapping_time_pub_ = nh.advertise<std_msgs::Float32>("/far_mapping_time", 5);

  //DEBUG Publisher
  dynamic_obs_pub_     = nh.advertise<sensor_msgs::PointCloud2>("/DP_dynamic_obs_debug",1);
  surround_free_debug_ = nh.advertise<sensor_msgs::PointCloud2>("/DP_free_debug",1);
  ground_pc_debug_ = nh.advertise<sensor_msgs::PointCloud2>("/DP_gnd_debug",1);
  surround_obs_debug_  = nh.advertise<sensor_msgs::PointCloud2>("/DP_obs_debug",1);
  scan_grid_debug_     = nh.advertise<sensor_msgs::PointCloud2>("/DP_scanGrid_debug",1);

  this->LoadROSParams();
  /* init Dynamic Planner Processing Objects */
  map_handler_.Init(map_params_);
  const int N_Layer = map_handler_.GetMapSize().z();
  DPUtil::layerIdx2Height_.resize(N_Layer);

  contour_detector_.Init(cdetect_params_);
  graph_manager_.Init(nh, graph_params_);
  graph_planner_.Init(nh, gp_params_);
  contour_graph_.Init(cg_params_);
  planner_viz_.Init(nh);
  scan_handler_.Init(scan_params_);
  graph_msger_.Init(nh, msger_parmas_);

  /* init internal params */
  odom_node_ptr_      = NULL;
  is_cloud_init_      = false;
  is_odom_init_       = false;
  is_scan_init_       = false;
  is_planner_running_ = false;
  is_goal_update_     = false;
  is_dyobs_update_    = false;
  is_graph_init_      = false;
  is_robot_stop_      = false;
  is_new_iter_        = false;
  is_reset_env_       = false;

  // allocate memory to pointers
  new_vertices_ptr_  = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
  temp_obs_ptr_      = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
  temp_free_ptr_     = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
  temp_cloud_ptr_    = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
  scan_grid_ptr_     = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
  local_terrian_ptr_ = PointCloudPtr(new pcl::PointCloud<PCLPoint>());

  // init global utility cloud
  DPUtil::new_obs_cloud_->clear();
  DPUtil::stack_dyobs_cloud_->clear();

  // init TF listener
  tf_listener_ = new tf::TransformListener();

  // clear temp vectors and memory
  this->ClearTempMemory();
  DPUtil::robot_pos = Point3D(0,0,0);
  DPUtil::free_odom_p = Point3D(0,0,0);

  robot_pos_   = Point3D(0,0,0);
  nav_heading_ = Point3D(0,0,0);
  nav_goal_    = Point3D(0,0,0);
  goal_waypoint_stamped_.header.frame_id = master_params_.world_frame;
  ROS_INFO_COND(master_params_.is_simulation, "********************************** SIMULATION PLANNING **********************************");
  ROS_INFO_COND(!master_params_.is_simulation, "********************************** REAL ROBOT PLANNING **********************************");
}

void DPMaster::ResetEnvironmentAndGraph() {
  this->ResetInternalValues();
  graph_manager_.ResetCurrentGraph();
  map_handler_.ResetGripMapCloud();
  graph_planner_.ResetPlannerInternalValues();
  /* Reset clouds */
  DPUtil::surround_obs_cloud_->clear();
  DPUtil::surround_free_cloud_->clear();
  DPUtil::new_obs_cloud_->clear();
  DPUtil::stack_dyobs_cloud_->clear();
  DPUtil::cur_dyobs_cloud_->clear();
  /* Stop the robot if it is moving */
  goal_waypoint_stamped_.header.stamp = ros::Time::now();
  goal_waypoint_stamped_.point = DPUtil::Point3DToGeoMsgPoint(robot_pos_);
  goal_pub_.publish(goal_waypoint_stamped_);
  PointStack empty_path;
  planner_viz_.VizPath(empty_path);
}


void DPMaster::Loop() {
  ros::Rate loop_rate(master_params_.main_run_freq);
  while (ros::ok()) {
    ros::spinOnce(); // Callback functions
    if (!this->PreconditionCheck()) {
      loop_rate.sleep();
      continue;
    }
    if (is_reset_env_) {
      this->ResetEnvironmentAndGraph(); 
      is_reset_env_ = false;
      ROS_WARN("**************************** Graph and Env Reset **********************************");
    }
    /* add main process after this line */
    map_handler_.UpdateRobotPosition(robot_pos_, cur_layer_idxs_);
    if (graph_manager_.UpdateOdom(robot_pos_, cur_layer_idxs_, is_robot_stop_) || is_goal_update_ || is_dyobs_update_ || !is_graph_init_) {
      odom_node_ptr_ = graph_manager_.GetOdomNode();
      if (odom_node_ptr_ == NULL) {
        ROS_WARN("DPMaster: wait for odom node to init...");
        loop_rate.sleep();
        continue;
      }
      this->ClearTempMemory();

      std_msgs::Float32 mapping_timer;
      DPUtil::Timer.start_time("Graph_mapping");
      /* Extract Vertices and new nodes */
      PointCloudPtr layer_cloud_ptr(new pcl::PointCloud<PCLPoint>());
      // ROS_ERROR("layer_idxs size: %ld", cur_layer_idxs_.size());
      for (const int& layer_idx : cur_layer_idxs_) {
        std::vector<PointStack> realworld_contour;
        map_handler_.GetSurroundObsCloud(layer_cloud_ptr, layer_idx);
        // DPUtil::Timer.start_time("Image_operation");        
        contour_detector_.BuildTerrianImgAndExtractContour(odom_node_ptr_, DPUtil::layerIdx2Height_[layer_idx], layer_cloud_ptr, realworld_contour);
        // DPUtil::Timer.end_time("Image_operation");
        if (realworld_contour.size() == 0) {
          ROS_WARN("DPMaster: No contour detected in layer %d", layer_idx);
          continue;
        }
        contour_graph_.UpdateContourGraph(odom_node_ptr_, layer_idx, realworld_contour);
        if (layer_idx != cur_layer_idxs_.front()) {
          contour_graph_.ConnectVerticalEdges(layer_idx-1);
        }
      }

      wide_nav_graph_ = graph_manager_.GetWideNavGraph();
      contour_graph_.MatchContourWithNavGraph(wide_nav_graph_, cur_layer_idxs_, new_ctnodes_);

      // DPUtil::ConvertCTNodeStackToPCL(new_ctnodes_, new_vertices_ptr_);
      // ROS_INFO("DPMaster: Vertices detected, number of vertices: %ld", new_vertices_ptr_->size());
      // planner_viz_.VizPointCloud(vertices_PCL_pub_, new_vertices_ptr_);
      
      // /* update planner graph */
      if (graph_manager_.ExtractGraphNodes(new_ctnodes_)) {
        new_nodes_ = graph_manager_.GetNewNodes();
        ROS_INFO("DPMaster: Potential nav nodes has been extracted: %ld", new_nodes_.size());
      }
      planner_viz_.VizNodes(new_nodes_, "new_nodes", VizColor::YELLOW);
      /* Graph Updating */
      graph_manager_.UpdateNavGraph(new_nodes_, clear_nodes_);
      mapping_timer.data = DPUtil::Timer.end_time("Graph_mapping");
      mapping_time_pub_.publish(mapping_timer);

      // // Visualize nodes
      planner_viz_.VizNodes(clear_nodes_, "clear_nodes", VizColor::ORANGE);
      const NavNodePtr last_internav_ptr = graph_manager_.GetLastInterNavNode();
      if (last_internav_ptr != NULL) {
        planner_viz_.VizPoint3D(last_internav_ptr->position, "last_nav_node", VizColor::MAGNA, 1.0);
      }
      nav_graph_ = graph_manager_.GetNavGraph();
      contour_graph_.ExtractGlobalContours(nav_graph_, cur_layer_idxs_);
      if (!is_graph_init_ && !ContourGraph::multi_global_contour_[odom_node_ptr_->layer_id].empty()) {
        is_graph_init_ = true;
        ROS_WARN("DPMaster: Navigation graph has been initialized.");
      }

      /* Graph Messager Update */
      graph_msger_.UpdateGlobalGraph(nav_graph_);
      // Graph Planner Update 
      graph_planner_.UpdateGlobalGraph(nav_graph_);
      /* Update connection of goal node if exist */
      goal_node_ptr_ = graph_planner_.GetGoalNodePtr();

      if (goal_node_ptr_ != NULL) {
        graph_planner_.insert_nav_nodes.clear();
        graph_planner_.UpdateGoalNavNodeConnects(goal_node_ptr_);
        graph_planner_.InsertNodes(odom_node_ptr_, goal_node_ptr_);
      }
      DPUtil::Timer.start_time("Path_planning");

      // /* Graph Planning */
      graph_planner_.UpdateGraphTraverability(odom_node_ptr_);
      // std::cout << "DPMaster: Graph traverability updated." << std::endl;
      // cout<<"ContourGraph::multi_contour_graph_: "<<ContourGraph::multi_contour_graph_.size()<<endl;
      PointStack global_path;
      Point3D last_nav_goal, current_free_goal;
      bool is_planning_fails = false;
      last_nav_goal = nav_goal_;
      goal_waypoint_stamped_.header.stamp = ros::Time::now();
      bool is_current_free_nav = false;
      NodePtrStack global_path_ptr;

      if (is_graph_init_ && graph_planner_.NextGoalPlanning(global_path, nav_goal_, current_free_goal, is_planning_fails, is_current_free_nav, global_path_ptr)) {
        graph_planner_.UpdateGraphTraverability(odom_node_ptr_);
        global_path.clear();
        global_path_ptr.clear();
        is_planning_fails = false;
        is_current_free_nav = false;
      }

      if (is_graph_init_ && graph_planner_.NextGoalPlanning(global_path, nav_goal_, current_free_goal, is_planning_fails, is_current_free_nav, global_path_ptr)) {
        graph_planner_.is_divided_path = false;
        Point3D waypoint = nav_goal_;
        if ((waypoint - current_free_goal).norm() > DPUtil::kEpsilon) {
          waypoint = this->ProjectNavWaypoint(waypoint, last_nav_goal);
        }
        goal_waypoint_stamped_.point = DPUtil::Point3DToGeoMsgPoint(waypoint);
        goal_pub_.publish(goal_waypoint_stamped_);
        is_planner_running_ = true;
        ROS_INFO("DPMaster: publishing waypoint...");
        planner_viz_.VizPoint3D(waypoint, "waypoint", VizColor::MAGNA, 1.5);
        planner_viz_.VizPoint3D(current_free_goal, "free_goal", VizColor::GREEN, 1.5);
        planner_viz_.VizPath(global_path, is_current_free_nav);
      } else if (is_planner_running_) {
        // stop robot
        global_path.clear();
        planner_viz_.VizPath(global_path);
        is_planner_running_ = false;
        nav_heading_ = Point3D(0,0,0);
        if (is_planning_fails) {
          // stops the robot
          goal_waypoint_stamped_.point = DPUtil::Point3DToGeoMsgPoint(robot_pos_);
          goal_pub_.publish(goal_waypoint_stamped_);
        }
      }
      DPUtil::Timer.end_time("Path_planning");
      std::cout << "DPMaster: Graph planning finished." << std::endl;

      planner_viz_.VizNodes(graph_planner_.insert_nav_nodes, "insert_nav_nodes", VizColor::GREEN);
      
      // Viz Navigation Graph
      // planner_viz_.VizPoint3D(DPUtil::free_odom_p, "free_odom_position", VizColor::ORANGE, 1.0);
      nav_graph_ = graph_planner_.GetNavGraph();
      planner_viz_.VizGraph(nav_graph_);

      // if (goal_node_ptr_ != NULL && DPUtil::IsTypeInStack(graph_planner_.insert_nav_nodes[0], graph_planner_.insert_nav_nodes[1]->connect_nodes)) {
      //   ROS_ERROR("DPMaster: Insert node is connected!");
      // }

      planner_viz_.VizContourGraph(ContourGraph::multi_contour_graph_, ContourGraph::multi_global_contour_, cur_layer_idxs_);
      graph_planner_.ClearInsertNodes(graph_planner_.insert_nav_nodes);
      graph_planner_.insert_nav_nodes.clear();


      
      // reset additional update flag
      if (is_robot_stop_) {
        ROS_WARN_COND(is_goal_update_,  "DPMaster: Goal updated, wait for robot to move...");
        ROS_WARN_COND(is_dyobs_update_, "DPMaster: Dynamic obstacle detected, wait for robot to move...");
      }
      is_goal_update_  = false;
      is_dyobs_update_ = false;
      is_new_iter_     = true;
    }
    /* reset and clear temped memory */
    loop_rate.sleep();
  }
}

Point3D DPMaster::ProjectNavWaypoint(const Point3D& nav_waypoint, const Point3D& last_waypoint) {
  const bool is_momentum = (last_waypoint - nav_waypoint).norm() < DPUtil::kEpsilon ? true : false; // momentum heading if same goal
  Point3D waypoint = nav_waypoint;
  const float dist = master_params_.waypoint_project_dist;
  const Point3D diff_p = nav_waypoint - robot_pos_;
  Point3D new_heading;
  if (is_momentum && nav_heading_.norm() > DPUtil::kEpsilon) {
    const float hdist = dist / 2.0;
    const float ratio = std::min(hdist, diff_p.norm()) / hdist;
    new_heading = diff_p.normalize() * ratio + nav_heading_ * (1.0 - ratio);
  } else {
    new_heading = diff_p.normalize();
  }
  if (nav_heading_.norm() > DPUtil::kEpsilon && new_heading.norm_dot(nav_heading_) < 0.0) { // negative direction reproject
    Point3D temp_heading(nav_heading_.y, -nav_heading_.x, nav_heading_.z);
    if (temp_heading.norm_dot(new_heading) < 0.0) {
      temp_heading.x = -temp_heading.x, temp_heading.y = -temp_heading.y;
    }
    new_heading = temp_heading;
  }
  nav_heading_ = new_heading.normalize();
  if (diff_p.norm() < dist) {
    waypoint = nav_waypoint + nav_heading_ * (dist - diff_p.norm());
  }
  return waypoint;
}

void DPMaster::LoadROSParams() {
  const std::string master_prefix   = "/airfar_planner/";
  const std::string map_prefix      = master_prefix + "MapHandler/";
  const std::string scan_prefix     = master_prefix + "ScanHandler/";
  const std::string cdetect_prefix  = master_prefix + "CDetector/";
  const std::string graph_prefix    = master_prefix + "Graph/";
  const std::string viz_prefix      = master_prefix + "Viz/";
  const std::string utility_prefix  = master_prefix + "Util/";
  const std::string planner_prefix  = master_prefix + "GPlanner/";
  const std::string contour_prefix  = master_prefix + "ContourGraph/";
  const std::string msger_prefix    = master_prefix + "GraphMsger/";

  // master params
  nh.param<float>(master_prefix + "main_run_freq", master_params_.main_run_freq, 5.0);
  nh.param<float>(master_prefix + "voxel_dim", master_params_.voxel_dim, 0.2);
  nh.param<float>(master_prefix + "robot_dim", master_params_.robot_dim, 0.5);
  nh.param<float>(master_prefix + "vehicle_height", master_params_.vehicle_height, 0.75);
  nh.param<float>(master_prefix + "sensor_range", master_params_.sensor_range, 100.0);
  nh.param<float>(master_prefix + "reproject_dist", master_params_.waypoint_project_dist, 5.0);
  nh.param<float>(master_prefix + "layer_resoltion", master_params_.layer_resolution, 1.0);
  nh.param<int>(master_prefix   + "neighbor_layer_num", master_params_.neighbor_layers, 2);
  nh.param<float>(master_prefix + "visualize_ratio", master_params_.viz_ratio, 1.0);
  nh.param<bool>(master_prefix  + "is_inter_navpoint", master_params_.is_inter_navpoint, true);
  nh.param<bool>(master_prefix  + "is_simulation", master_params_.is_simulation, true);
  nh.param<bool>(master_prefix  + "is_trajectory", master_params_.is_trajectory_edge, false);
  nh.param<bool>(master_prefix  + "attempt_autoswitch", master_params_.is_attempt_autoswitch, false);
  nh.param<std::string>(master_prefix + "world_frame", master_params_.world_frame, "map");

  // map handler params
  nh.param<float>(map_prefix + "ceil_length", map_params_.ceil_length, 5.0);
  nh.param<float>(map_prefix + "map_grid_max_length", map_params_.grid_max_length, 5000.0);
  nh.param<float>(map_prefix + "map_grad_max_height", map_params_.grid_max_height, 100.0);
  map_params_.neighbor_layers = master_params_.neighbor_layers;
  map_params_.ceil_height = master_params_.layer_resolution;
  map_params_.sensor_range = master_params_.sensor_range;

  // graph messager params
  nh.param<int>(msger_prefix + "robot_id", msger_parmas_.robot_id, 0);
  msger_parmas_.frame_id = master_params_.world_frame;

  // graph planner params
  nh.param<float>(planner_prefix + "converge_distance", gp_params_.converge_dist, 1.0);
  nh.param<float>(planner_prefix + "goal_adjust_radius", gp_params_.adjust_radius, 10.0);
  nh.param<float>(planner_prefix + "goal_z_tolerance", gp_params_.goal_z_tolerance, 2.0);
  nh.param<int>(planner_prefix   + "free_counter_thred", gp_params_.free_thred, 5);
  nh.param<float>(planner_prefix + "free_box_dim", gp_params_.free_box_dim, 1.0);
  nh.param<int>(planner_prefix   + "reach_goal_vote_size", gp_params_.votes_size, 5);
  nh.param<float>(planner_prefix + "path_momentum_ratio", gp_params_.momentum_dist, 1.0);
  nh.param<int>(planner_prefix   + "path_momentum_thred", gp_params_.momentum_thred, 5);
  nh.param<int>(planner_prefix   + "clear_inflate_size", gp_params_.clear_inflate_size, 3);
  gp_params_.is_autoswitch = master_params_.is_attempt_autoswitch;

  // dynamic graph params
  nh.param<float>(graph_prefix  + "node_near_dist", graph_params_.near_dist, 2.0);
  nh.param<float>(graph_prefix  + "move_thred", graph_params_.move_thred, 0.25);
  nh.param<int>(graph_prefix    + "terrain_inflate_size", graph_params_.terrian_inflate, 2);
  nh.param<int>(graph_prefix    + "connect_votes_size", graph_params_.votes_size, 10);
  nh.param<int>(graph_prefix    + "clear_dumper_thred", graph_params_.dumper_thred, 3);
  nh.param<int>(graph_prefix    + "node_finalize_thred", graph_params_.finalize_thred, 3);
  nh.param<int>(graph_prefix    + "RANSAC_pool_size", graph_params_.pool_size, 12);
  nh.param<float>(graph_prefix  + "connect_angle_thred", graph_params_.kConnectAngleThred, 10.0);
  nh.param<float>(graph_prefix  + "trajectory_interval_ratio", graph_params_.traj_interval_ratio, 2.0);
  nh.param<float>(graph_prefix  + "pos_filter_margin", graph_params_.filter_pos_margin, 0.5);
  nh.param<float>(graph_prefix  + "dirs_filter_margin", graph_params_.filter_dirs_margin, 10.0);
  graph_params_.kConnectAngleThred = graph_params_.kConnectAngleThred / 180.0 * M_PI;
  graph_params_.filter_dirs_margin = graph_params_.filter_dirs_margin / 180.0 * M_PI;
  graph_params_.sensor_range = master_params_.sensor_range;

  // utility params
  nh.param<float>(utility_prefix + "new_intensity_thred", DPUtil::kNewPIThred, 2.0);
  nh.param<float>(utility_prefix + "nav_clear_dist", DPUtil::kNavClearDist, 0.5);
  nh.param<float>(utility_prefix + "terrain_free_Z", DPUtil::kFreeZ, 0.1);
  nh.param<float>(utility_prefix + "dynamic_obs_dacay_time", DPUtil::kDecayTime, 10.0);
  nh.param<int>(utility_prefix   + "dyosb_update_thred", DPUtil::kDyObsThred, 4);
  nh.param<int>(utility_prefix   + "new_point_counter", DPUtil::KNewPointC, 10);
  nh.param<float>(utility_prefix + "angle_noise", DPUtil::kAngleNoise, 15.0);
  nh.param<float>(utility_prefix + "vertical_angle_thred", DPUtil::kVerticalAngleThred, 30.0);
  nh.param<float>(utility_prefix + "accept_max_align_angle", DPUtil::kAcceptAlign, 15.0);
  // convert to radians
  DPUtil::worldFrameId    = master_params_.world_frame;
  DPUtil::kVizRatio       = master_params_.viz_ratio;
  DPUtil::kTolerZ         = map_params_.ceil_height * (master_params_.neighbor_layers + 0.5);
  DPUtil::kCellLength     = map_params_.ceil_length;
  DPUtil::kAcceptAlign    = DPUtil::kAcceptAlign / 180.0 * M_PI;
  DPUtil::kAngleNoise     = DPUtil::kAngleNoise / 180.0 * M_PI; 
  DPUtil::kVerticalAngleThred = DPUtil::kVerticalAngleThred / 180.0 * M_PI;
  DPUtil::robot_dim = master_params_.robot_dim;
  DPUtil::IsNavpoint = master_params_.is_inter_navpoint;
  DPUtil::IsSimulation = master_params_.is_simulation;
  DPUtil::IsTrajectory = master_params_.is_trajectory_edge;
  DPUtil::vehicle_height = master_params_.vehicle_height;
  DPUtil::kLeafSize = master_params_.voxel_dim;
  DPUtil::kNearDist = graph_params_.near_dist;
  DPUtil::kSensorRange = master_params_.sensor_range;
  DPUtil::kNeighborLayers = master_params_.neighbor_layers;

  // scan handler params
  nh.param<int>(scan_prefix + "inflate_scan_size", scan_params_.inflate_size, 2);
  scan_params_.sensor_range = master_params_.sensor_range;
  scan_params_.voxel_size = master_params_.voxel_dim;
  scan_params_.ceil_height = DPUtil::kTolerZ * 4.0;

  // contour detector params
  nh.param<bool>(cdetect_prefix        + "is_inflate_image", cdetect_params_.is_inflate, true);
  nh.param<bool>(cdetect_prefix        + "is_save_img", cdetect_params_.is_save_img, false);
  nh.param<float>(cdetect_prefix       + "resize_ratio", cdetect_params_.kRatio, 5.0);
  nh.param<double>(cdetect_prefix       + "approx_eps", cdetect_params_.approx_eps, 0.0005);
  nh.param<int>(cdetect_prefix         + "filter_count_value", cdetect_params_.kThredValue, 10);
  nh.param<int>(cdetect_prefix         + "img_blur_size", cdetect_params_.kBlurSize, 10);
  nh.param<std::string>(cdetect_prefix + "img_folder_path", cdetect_params_.img_path, "");
  cdetect_params_.sensor_range = master_params_.sensor_range;
  cdetect_params_.voxel_dim = master_params_.voxel_dim;

  // contour graph params
  nh.param<float>(contour_prefix + "around_distance", cg_params_.kAroundDist, 1.0);
  nh.param<float>(contour_prefix + "pillar_perimeter_dist", cg_params_.kPillarPerimeter, 4.0);
  nh.param<int>(contour_prefix + "KDTree_num", cg_params_.KD_TREE_K, 5);
  nh.param<int>(contour_prefix + "KDTree_neighbor_size", cg_params_.knn_search_num_, 1);
  nh.param<float>(contour_prefix + "KNN_search_radius", cg_params_.knn_search_radius_, 1.5);
  nh.param<float>(contour_prefix + "wall_insert_factor", cg_params_.wall_insert_factor, 0.6);
  cg_params_.voxel_dim = master_params_.voxel_dim;
  graph_params_.wall_insert_factor = cg_params_.wall_insert_factor;

}

void DPMaster::OdomCallBack(const nav_msgs::OdometryConstPtr& msg) {
  // transform from odom frame to mapping frame
  std::string odom_frame = msg->header.frame_id;
  tf::Pose tf_odom_pose;
  tf::poseMsgToTF(msg->pose.pose, tf_odom_pose);
  if (!DPUtil::IsSameFrameID(odom_frame, master_params_.world_frame)) {
    ROS_WARN_COND(!is_odom_init_,"DPMaster: odom frame does NOT match with world frame!");
    tf::StampedTransform odom_to_world_tf_stamp;
    try
    {
      tf_listener_->waitForTransform(master_params_.world_frame, odom_frame, ros::Time(0), ros::Duration(1.0));
      tf_listener_->lookupTransform(master_params_.world_frame, odom_frame, ros::Time(0), odom_to_world_tf_stamp);
      tf_odom_pose = odom_to_world_tf_stamp * tf_odom_pose;
    }
    catch (tf::TransformException ex){
      ROS_ERROR("Tracking odom TF lookup: %s",ex.what());
      return;
    }
  }
  robot_pos_.x = tf_odom_pose.getOrigin().getX(); 
  robot_pos_.y = tf_odom_pose.getOrigin().getY();
  robot_pos_.z = tf_odom_pose.getOrigin().getZ();
  // extract robot heading
  DPUtil::robot_pos = robot_pos_;
  double roll, pitch, yaw;
  tf_odom_pose.getBasis().getRPY(roll, pitch, yaw);
  robot_heading_ = Point3D(cos(yaw), sin(yaw), 0);

  if (!is_odom_init_) {
    // system start time
    DPUtil::systemStartTime = ros::Time::now().toSec();
    DPUtil::map_origin = robot_pos_;
    map_handler_.UpdateRobotPosition(robot_pos_, cur_layer_idxs_);
  }

  is_odom_init_ = true;
}

void DPMaster::PrcocessCloud(const sensor_msgs::PointCloud2ConstPtr& pc,
                             const PointCloudPtr& cloudOut,
                             const bool& is_crop_cloud) 
{
  pcl::PointCloud<PCLPoint> temp_cloud;
  pcl::fromROSMsg(*pc, temp_cloud);
  cloudOut->clear(), *cloudOut = temp_cloud;
  if (cloudOut->empty()) return;
  DPUtil::FilterCloud(cloudOut, master_params_.voxel_dim);
  // transform cloud frame
  std::string cloud_frame = pc->header.frame_id;
  DPUtil::RemoveNanInfPoints(cloudOut);
  if (!DPUtil::IsSameFrameID(cloud_frame, master_params_.world_frame)) {
    ROS_WARN_ONCE("DPMaster: cloud frame does NOT match with world frame!");
    try
    {
      DPUtil::TransformPCLFrame(cloud_frame, 
                                master_params_.world_frame, 
                                tf_listener_,
                                cloudOut);
    }
    catch(tf::TransformException ex)
    {
      ROS_ERROR("Tracking cloud TF lookup: %s",ex.what());
      return;
    }
  }
    planner_viz_.VizPointCloud(ground_pc_debug_,  cloudOut);  
  if (is_crop_cloud) {
    const Point3D crop_size(master_params_.sensor_range, master_params_.sensor_range, DPUtil::kTolerZ);
    DPUtil::CropBoxCloud(cloudOut, robot_pos_, crop_size);
  }

}

void DPMaster::ScanCallBack(const sensor_msgs::PointCloud2ConstPtr& scan_pc) {
  if (!is_odom_init_) return;
  this->PrcocessCloud(scan_pc, DPUtil::cur_scan_cloud_, true);
  scan_handler_.UpdateRobotPosition(robot_pos_);
}

void DPMaster::TerrainLocalCallBack(const sensor_msgs::PointCloud2ConstPtr& pc) {
  if (!master_params_.is_inter_navpoint || !master_params_.is_trajectory_edge) return;
  this->PrcocessCloud(pc, local_terrian_ptr_, false);
  // DPUtil::ExtractFreeAndObsCloud(local_terrian_ptr_, DPUtil::local_terrain_free_, DPUtil::local_terrain_obs_);

}

void DPMaster::TerrainCallBack(const sensor_msgs::PointCloud2ConstPtr& pc) {
  if (!is_odom_init_) {
    ROS_WARN_THROTTLE(1.0, "DPMaster: wait for odom to init.");
    return;
  }
  if (!is_robot_stop_ || !is_graph_init_) {
    this->PrcocessCloud(pc, temp_cloud_ptr_, true);
    DPUtil::ExtractFreeAndObsFromScanCloud(DPUtil::cur_scan_cloud_, temp_cloud_ptr_, temp_free_ptr_, temp_obs_ptr_);
    if (!master_params_.is_simulation) {
      DPUtil::RemoveOverlapCloud(temp_obs_ptr_, DPUtil::stack_dyobs_cloud_, true);
    }
    map_handler_.UpdateObsCloudGrid(temp_obs_ptr_);
    map_handler_.UpdateFreeCloudGrid(temp_free_ptr_);
    // extract new points
    DPUtil::ExtractNewObsPointCloud(temp_obs_ptr_,
                                    DPUtil::surround_obs_cloud_,
                                    DPUtil::new_obs_cloud_, !is_new_iter_);
    is_new_iter_ = false;
 
    // extract surround cloud
    map_handler_.GetSurroundObsCloud(DPUtil::surround_obs_cloud_);
    map_handler_.GetSurroundFreeCloud(DPUtil::surround_free_cloud_);
  }
  // extract dynamic obstacles
  DPUtil::cur_dyobs_cloud_->clear();
  if (!master_params_.is_simulation) {
    this->ExtractDynamicObsFromScan(DPUtil::cur_scan_cloud_, DPUtil::surround_obs_cloud_, DPUtil::cur_dyobs_cloud_);
    if (DPUtil::cur_dyobs_cloud_->size() > DPUtil::kDyObsThred) {
      ROS_WARN("DPMaster: dynamic obstacle detected, size: %ld", DPUtil::cur_dyobs_cloud_->size());
      DPUtil::InflateCloud(DPUtil::cur_dyobs_cloud_, master_params_.voxel_dim, 1, true);
      map_handler_.RemoveObsCloudFromGrid(DPUtil::cur_dyobs_cloud_);
      DPUtil::RemoveOverlapCloud(DPUtil::surround_obs_cloud_, DPUtil::cur_dyobs_cloud_);
      DPUtil::FilterCloud(DPUtil::cur_dyobs_cloud_, master_params_.voxel_dim);
      // update new cloud
      *DPUtil::new_obs_cloud_ += *DPUtil::cur_dyobs_cloud_;
      DPUtil::FilterCloud(DPUtil::new_obs_cloud_, master_params_.voxel_dim);
      is_dyobs_update_ = true;
    }
    // update world dynamic obstacles
    DPUtil::StackCloudByTime(DPUtil::cur_dyobs_cloud_, DPUtil::stack_dyobs_cloud_, DPUtil::kDecayTime);
  }
  
  // create and update kdtrees
  if (!is_robot_stop_ || !is_graph_init_ || !DPUtil::cur_dyobs_cloud_->empty()) {
    DPUtil::UpdateKdTrees(DPUtil::new_obs_cloud_, DPUtil::surround_free_cloud_, DPUtil::surround_obs_cloud_);
  }
  if (!DPUtil::surround_obs_cloud_->empty()) is_cloud_init_ = true;

  /* visualize clouds */
  planner_viz_.VizPointCloud(new_PCL_pub_,         DPUtil::new_obs_cloud_);
  planner_viz_.VizPointCloud(dynamic_obs_pub_,     DPUtil::cur_dyobs_cloud_);
  if (!is_robot_stop_ || !is_graph_init_) {
    planner_viz_.VizPointCloud(surround_free_debug_, DPUtil::surround_free_cloud_);
    planner_viz_.VizPointCloud(surround_obs_debug_,  DPUtil::surround_obs_cloud_);
    // planner_viz_.VizPointCloud(ground_pc_debug_,  DPUtil::cur_scan_cloud_);
    // visualize map grid
    PointStack neighbor_centers, occupancy_centers;
    map_handler_.GetNeighborCeilsCenters(neighbor_centers);
    map_handler_.GetOccupancyCeilsCenters(occupancy_centers);
    planner_viz_.VizMapGrids(neighbor_centers, occupancy_centers, map_params_.ceil_length, map_params_.ceil_height);
  }
  // DBBUG visual raycast grids
  if (!master_params_.is_simulation) {
    scan_handler_.GridVisualCloud(scan_grid_ptr_, GridStatus::RAY);
    planner_viz_.VizPointCloud(scan_grid_debug_, scan_grid_ptr_);
  }
}

void DPMaster::ExtractDynamicObsFromScan(const PointCloudPtr& scanCloudIn, 
                                         const PointCloudPtr& obsCloudIn, 
                                         const PointCloudPtr& dyObsCloudOut)
{
  scan_handler_.ReInitGrids();
  scan_handler_.SetCurrentScanCloud(scanCloudIn);
  scan_handler_.ExtractDyObsCloud(obsCloudIn, dyObsCloudOut);
}

void DPMaster::WaypointCallBack(const geometry_msgs::PointStampedConstPtr & msg) {
  Point3D goal_p(msg->point.x, msg->point.y, msg->point.z);
  const std::string goal_frame = msg->header.frame_id;
  if (!DPUtil::IsSameFrameID(goal_frame, master_params_.world_frame)) {
    ROS_WARN("DPMaster: waypoint published is not on world frame!");
    DPUtil::TransformPoint3DFrame(goal_frame, master_params_.world_frame, tf_listener_, goal_p); 
  }
  const int layer_id = map_handler_.GetLayerId(goal_p);
  goal_p.z = DPUtil::layerIdx2Height_[layer_id];
  if (layer_id != -1) {
    graph_planner_.UpdateGoal(goal_p, layer_id, false);
    is_goal_update_  = true;
    // visualize original goal
    // planner_viz_.VizPoint3D(goal_node_ptr_->position, "goal_node", VizColor::BLUE, 2.5);
    planner_viz_.VizPoint3D(goal_p, "original_goal", VizColor::RED, 1.5);
  } else {
    ROS_WARN("DPMaster: goal layer id assign failed.");
  }
}

void DPMaster::TargetCallBack(const geometry_msgs::PoseStampedConstPtr & msg) {
  Point3D goal_p(msg->pose.position.x, msg->pose.position.y, 2.0);
  const std::string goal_frame = msg->header.frame_id;
  if (!DPUtil::IsSameFrameID(goal_frame, master_params_.world_frame)) {
    ROS_WARN("DPMaster: target pose published is not on world frame!");
    DPUtil::TransformPoint3DFrame(goal_frame, master_params_.world_frame, tf_listener_, goal_p); 
  }
  const int layer_id = map_handler_.GetLayerId(goal_p);
  goal_p.z = DPUtil::layerIdx2Height_[layer_id];
  if (layer_id != -1) {
    graph_planner_.UpdateGoal(goal_p, layer_id, true);
    is_goal_update_  = true;
    // visualize original goal
    // planner_viz_.VizPoint3D(goal_node_ptr_->position, "goal_node", VizColor::BLUE, 2.5);
    planner_viz_.VizPoint3D(goal_p, "original_goal", VizColor::RED, 1.5);
  } else {
    ROS_WARN("DPMaster: goal layer id assign failed.");
  }
}

/* allocate static utility PointCloud pointer memory */
PointCloudPtr  DPUtil::surround_obs_cloud_  = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointCloudPtr  DPUtil::surround_free_cloud_ = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointCloudPtr  DPUtil::new_obs_cloud_       = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointCloudPtr  DPUtil::cur_dyobs_cloud_     = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointCloudPtr  DPUtil::stack_dyobs_cloud_   = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointCloudPtr  DPUtil::cur_scan_cloud_      = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointCloudPtr  DPUtil::free_flat_cloud_     = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointCloudPtr  DPUtil::local_terrain_obs_   = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointCloudPtr  DPUtil::local_terrain_free_  = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointKdTreePtr DPUtil::kdtree_new_cloud_       = PointKdTreePtr(new pcl::KdTreeFLANN<PCLPoint>());
PointKdTreePtr DPUtil::kdtree_filter_cloud_    = PointKdTreePtr(new pcl::KdTreeFLANN<PCLPoint>());
/* init static utility values */
const float DPUtil::kEpsilon = 1e-5;
const float DPUtil::kINF     = std::numeric_limits<float>::max();
std::string DPUtil::worldFrameId;
float   DPUtil::kAngleNoise; 
Point3D DPUtil::robot_pos;
Point3D DPUtil::odom_pos;
Point3D DPUtil::map_origin;
Point3D DPUtil::free_odom_p;
float   DPUtil::robot_dim;
float   DPUtil::vehicle_height;
float   DPUtil::kLeafSize;
float   DPUtil::kNavClearDist;
float   DPUtil::kCellLength;
float   DPUtil::kNewPIThred;
float   DPUtil::kSensorRange;
float   DPUtil::kFreeZ;
float   DPUtil::kVizRatio;
double  DPUtil::systemStartTime;
float   DPUtil::kDecayTime;
float   DPUtil::kNearDist;
int     DPUtil::kDyObsThred;
int     DPUtil::KNewPointC;
int     DPUtil::kNeighborLayers;
float   DPUtil::kVerticalAngleThred;
float   DPUtil::kTolerZ;
float   DPUtil::kAcceptAlign;
bool    DPUtil::IsNavpoint;
bool    DPUtil::IsSimulation;
bool    DPUtil::IsTrajectory;
TimeMeasure DPUtil::Timer;
std::vector<float> DPUtil::layerIdx2Height_; 

/* Global Graph */
NodePtrStack DynamicGraph::globalGraphNodes_;
std::size_t  DynamicGraph::id_tracker_;
float        DynamicGraph::VERTICAL_ANGLE_COS;

/* init static contour graph values */
std::vector<CTNodeStack>  ContourGraph::multi_contour_graph_;
std::vector<PolygonStack> ContourGraph::multi_contour_polygons_;
std::vector<std::vector<PointPair>> ContourGraph::multi_global_contour_;

int main(int argc, char** argv){
  ros::init(argc, argv, "airfar_planner_node");
  DPMaster dp_node;
  dp_node.Init();
  dp_node.Loop();
}