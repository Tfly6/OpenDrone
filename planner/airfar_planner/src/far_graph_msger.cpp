/*
 * FAR Planner
 * Copyright (C) 2021 Fan Yang - All rights reserved
 * fanyang2@andrew.cmu.edu,   
 */



#include "airfar_planner/graph_msger_far.h"

/***************************************************************************************/

void GraphMsgerFAR::Init(const ros::NodeHandle& nh, const GraphMsgerFARParams& params) {
    nh_ = nh;
    gm_params_ = params;
    graph_pub_ = nh_.advertise<airfar_planner::VisGraph>("/robot_vgraph", 5);
    graph_sub_ = nh_.subscribe("/decoded_vgraph", 5, &GraphMsgerFAR::GraphCallBack, this);

    global_graph_.clear();
    nodes_cloud_ptr_    = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
    kdtree_graph_cloud_ = PointKdTreePtr(new pcl::KdTreeFLANN<PCLPoint>());
    kdtree_graph_cloud_->setSortedResults(false);
}
