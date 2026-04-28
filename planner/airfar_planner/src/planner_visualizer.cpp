/*
 * Dynamic Route Planner
 * Copyright (C) 2021 Fan Yang - All rights reserved
 * fanyang2@andrew.cmu.edu,   
 */



#include "airfar_planner/planner_visualizer.h"

/***************************************************************************************/


void DPVisualizer::Init(const ros::NodeHandle& nh) {
    nh_ = nh;
    point_cloud_ptr_ = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
    // Rviz Publisher
    viz_node_pub_    = nh_.advertise<Marker>("/viz_node_topic", 5);
    viz_path_pub_    = nh_.advertise<Marker>("/viz_path_topic", 5);
    viz_graph_pub_   = nh_.advertise<MarkerArray>("/viz_graph_topic", 5);
    viz_contour_pub_ = nh_.advertise<MarkerArray>("/viz_contour_topic", 5);
    viz_map_pub_     = nh_.advertise<MarkerArray>("/viz_grid_map_topic", 5);
    
}

void DPVisualizer::VizNodes(const NodePtrStack& node_stack, 
                            const std::string& ns,
                            const VizColor& color,
                            const float scale,
                            const float alpha)
{
    Marker node_marker;
    node_marker.type = Marker::SPHERE_LIST;
    this->SetMarker(color, ns, scale, alpha, node_marker);
    node_marker.points.resize(node_stack.size());
    std::size_t idx = 0;
    for (const auto& node_ptr : node_stack) {
        if (node_ptr == NULL) continue;
        node_marker.points[idx] = DPUtil::Point3DToGeoMsgPoint(node_ptr->position);
        idx ++;
    }
    node_marker.points.resize(idx);
    viz_node_pub_.publish(node_marker);
}

void DPVisualizer::VizPoint3D(const Point3D& point, 
                             const std::string& ns,
                             const VizColor& color,
                             const float scale,
                             const float alpha)
{
    Marker node_marker;
    node_marker.type = Marker::SPHERE;
    this->SetMarker(color, ns, scale, alpha, node_marker);
    std::size_t idx = 0;
    node_marker.pose.position.x = point.x;
    node_marker.pose.position.y = point.y;
    node_marker.pose.position.z = point.z;
    viz_node_pub_.publish(node_marker);
}

void DPVisualizer::VizPath(const PointStack& global_path, const bool& is_free_nav) {
    Marker path_marker;
    path_marker.type = Marker::LINE_STRIP;
    const VizColor color = is_free_nav ? VizColor::GREEN : VizColor::BLUE;
    this->SetMarker(color, "global_path", 0.75f, 0.9f, path_marker);
    geometry_msgs::Point geo_p;
    for (const auto& p : global_path) {
        geo_p = DPUtil::Point3DToGeoMsgPoint(p);
        path_marker.points.push_back(geo_p);
    }
    viz_path_pub_.publish(path_marker);
}

void DPVisualizer::VizContourGraph(const std::vector<CTNodeStack>& contour_graph,
                                   const std::vector<std::vector<PointPair>>& global_contour,
                                   const std::vector<int>& cur_layer_idxs) 
{
    MarkerArray contour_marker_array;
    Marker contour_vertex_marker, vertex_matched_marker, contour_marker, contour_surf_marker, contour_helper_marker, top_layer_vertex_marker, wall_corner_marker;
    Marker global_contour_marker;
    contour_vertex_marker.type = Marker::SPHERE_LIST;
    vertex_matched_marker.type = Marker::SPHERE_LIST;
    top_layer_vertex_marker.type = Marker::SPHERE_LIST;
    wall_corner_marker.type = Marker::SPHERE_LIST;
    contour_marker.type   = Marker::LINE_LIST;
    contour_surf_marker.type    = Marker::LINE_LIST;
    contour_helper_marker.type  = Marker::CUBE_LIST;
    global_contour_marker.type = Marker::LINE_LIST;
    this->SetMarker(VizColor::EMERALD, "contour_vertex", 0.5f, 0.5f, contour_vertex_marker);
    this->SetMarker(VizColor::RED,     "matched_vertex", 0.5f, 0.5f, vertex_matched_marker);
    this->SetMarker(VizColor::YELLOW, "top_layer_vertex", 0.5f, 0.5f, top_layer_vertex_marker);
    this->SetMarker(VizColor::YELLOW, "wall_corner_marker", 0.5f, 0.5f, wall_corner_marker);
    this->SetMarker(VizColor::MAGNA,   "contour",        0.1f, 0.25f, contour_marker);
    this->SetMarker(VizColor::BLUE,    "contour_surf",   0.15f, 0.75f, contour_surf_marker);
    this->SetMarker(VizColor::BLUE,    "contour_direct", 0.25f, 0.75f, contour_helper_marker);
    this->SetMarker(VizColor::WHITE,   "global_contour", 0.1f, 0.5f, global_contour_marker);

    auto Draw_Contour = [&](const CTNodePtr& ctnode_ptr) {
        geometry_msgs::Point geo_vertex, geo_connect;
        geo_vertex = DPUtil::Point3DToGeoMsgPoint(ctnode_ptr->position);
        contour_vertex_marker.points.push_back(geo_vertex);
        if (ctnode_ptr->is_global_match) {
            vertex_matched_marker.points.push_back(geo_vertex);
        }
        if (ctnode_ptr->up == NULL) {
            top_layer_vertex_marker.points.push_back(geo_vertex);
        }
        if (ctnode_ptr->front == NULL || ctnode_ptr->back == NULL) return;
        contour_marker.points.push_back(geo_vertex);
        geo_connect = DPUtil::Point3DToGeoMsgPoint(ctnode_ptr->front->position);
        contour_marker.points.push_back(geo_connect);

        contour_marker.points.push_back(geo_vertex);
        geo_connect = DPUtil::Point3DToGeoMsgPoint(ctnode_ptr->back->position);
        contour_marker.points.push_back(geo_connect);

        if (ctnode_ptr->up != NULL) {
            contour_marker.points.push_back(geo_vertex);
            geo_connect = DPUtil::Point3DToGeoMsgPoint(ctnode_ptr->up->position);
            contour_marker.points.push_back(geo_connect);
        }

        if (ctnode_ptr->is_wall_corner) {
            wall_corner_marker.points.push_back(geo_vertex);
        }
        // if (ctnode_ptr->down != NULL) {
        //     contour_marker.points.push_back(geo_vertex);
        //     geo_connect = DPUtil::Point3DToGeoMsgPoint(ctnode_ptr->down->position);
        //     contour_marker.points.push_back(geo_connect);
        // }

    };
    auto Draw_Surf_Dir = [&](const CTNodePtr& ctnode) {
        geometry_msgs::Point p1, p2, p3;
        p1 = DPUtil::Point3DToGeoMsgPoint(ctnode->position);
        Point3D end_p;
        if (ctnode->free_direct != NodeFreeDirect::PILLAR) {
            end_p = ctnode->position + ctnode->surf_dirs.first * DPUtil::kVizRatio;
            p2 = DPUtil::Point3DToGeoMsgPoint(end_p);
            contour_surf_marker.points.push_back(p1);
            contour_surf_marker.points.push_back(p2);
            contour_helper_marker.points.push_back(p2);
            end_p = ctnode->position + ctnode->surf_dirs.second * DPUtil::kVizRatio;
            p3 = DPUtil::Point3DToGeoMsgPoint(end_p);
            contour_surf_marker.points.push_back(p1);
            contour_surf_marker.points.push_back(p3);
            contour_helper_marker.points.push_back(p3);
        }
    };
    auto Draw_RemoveEdge = [&](const PointPair& pair) {
        geometry_msgs::Point p1, p2;
        p1 = DPUtil::Point3DToGeoMsgPoint(pair.first);
        p2 = DPUtil::Point3DToGeoMsgPoint(pair.second);
        global_contour_marker.points.push_back(p1);
        global_contour_marker.points.push_back(p2);
    };
    for (const int& layer_id : cur_layer_idxs) {
        for (const auto& ctnode : contour_graph[layer_id]) {
            if (ctnode == NULL) {
                ROS_ERROR("Viz: contour node is NULL.");
                continue;
            }
            Draw_Contour(ctnode);
            Draw_Surf_Dir(ctnode);
        }
        for (const auto& pair : global_contour[layer_id]) {
            Draw_RemoveEdge(pair);
        }
    }
    contour_marker_array.markers.push_back(contour_vertex_marker);
    contour_marker_array.markers.push_back(vertex_matched_marker);
    contour_marker_array.markers.push_back(contour_marker);
    contour_marker_array.markers.push_back(contour_surf_marker);
    contour_marker_array.markers.push_back(contour_helper_marker);
    contour_marker_array.markers.push_back(global_contour_marker);
    contour_marker_array.markers.push_back(top_layer_vertex_marker);
    contour_marker_array.markers.push_back(wall_corner_marker);
    viz_contour_pub_.publish(contour_marker_array);
}

void DPVisualizer::VizGraph(const NodePtrStack& graph) {
    MarkerArray graph_marker_array;
    Marker nav_node_marker, unfinal_node_marker, reachable_node_marker, near_node_marker, covered_node_marker, internav_node_marker, 
           edge_marker, contour_edge_marker, odom_edge_marker, goal_edge_marker, trajectory_edge_marker,
           corner_surf_marker, contour_align_marker, corner_helper_marker, top_node_marker, top_contour_marker, bottom_contour_marker, insert_node_marker;
    nav_node_marker.type       = Marker::SPHERE_LIST;
    unfinal_node_marker.type   = Marker::SPHERE_LIST;
    reachable_node_marker.type = Marker::SPHERE_LIST;
    near_node_marker.type      = Marker::SPHERE_LIST;
    covered_node_marker.type   = Marker::SPHERE_LIST;
    internav_node_marker.type  = Marker::SPHERE_LIST;
    top_node_marker.type       = Marker::SPHERE_LIST;
    insert_node_marker.type    = Marker::SPHERE_LIST;
    contour_align_marker.type  = Marker::LINE_LIST;
    edge_marker.type           = Marker::LINE_LIST;
    contour_edge_marker.type   = Marker::LINE_LIST;
    odom_edge_marker.type      = Marker::LINE_LIST;
    goal_edge_marker.type      = Marker::LINE_LIST;
    trajectory_edge_marker.type = Marker::LINE_LIST;
    corner_surf_marker.type    = Marker::LINE_LIST;
    top_contour_marker.type    = Marker::LINE_LIST;
    bottom_contour_marker.type = Marker::LINE_LIST;
    corner_helper_marker.type  = Marker::CUBE_LIST;
    this->SetMarker(VizColor::WHITE,   "graph_node",     0.5f,  0.5f,  nav_node_marker);
    this->SetMarker(VizColor::RED,     "unfinal_node",   0.5f,  0.8f,  unfinal_node_marker);
    this->SetMarker(VizColor::GREEN,   "reachable_node", 0.5f,  0.8f,  reachable_node_marker);
    this->SetMarker(VizColor::MAGNA,   "near_node",      0.5f,  0.8f,  near_node_marker);
    this->SetMarker(VizColor::BLUE,    "covered_nodes",  0.5f,  0.8f,  covered_node_marker);
    this->SetMarker(VizColor::YELLOW,  "internav_node",  0.5f,  0.8f,  internav_node_marker);
    this->SetMarker(VizColor::EMERALD, "graph_edge",     0.1f,  0.2f,  edge_marker);
    this->SetMarker(VizColor::YELLOW,  "contour_edge",   0.2f,  0.25f, contour_edge_marker);
    this->SetMarker(VizColor::ORANGE,  "odom_edge",      0.1f,  0.15f, odom_edge_marker);
    this->SetMarker(VizColor::PURPLE,  "goal_edge",      0.1f,  0.15f, goal_edge_marker);
    this->SetMarker(VizColor::GREEN,   "trajectory_edge",0.1f,  0.5f,  trajectory_edge_marker);
    this->SetMarker(VizColor::YELLOW,  "corner_surf",    0.15f, 0.75f, corner_surf_marker);
    this->SetMarker(VizColor::YELLOW,  "surf_direct",    0.25f, 0.75f, corner_helper_marker);
    this->SetMarker(VizColor::RED,     "contour_align",  0.1f,  0.5f,  contour_align_marker);
    this->SetMarker(VizColor::RED,     "top_node",       0.5f,  0.5f,  top_node_marker);
    this->SetMarker(VizColor::GREEN,   "top_contour",    0.2f,  0.2f, top_contour_marker);
    this->SetMarker(VizColor::WHITE,   "bottom_contour", 0.2f,  0.2f, bottom_contour_marker);
    this->SetMarker(VizColor::RED,  "insert_node",    0.5f,  0.5f,  insert_node_marker);

    /* Lambda Function */
    auto Draw_Contour_Align = [&](const NavNodePtr& node_ptr) {
        if (node_ptr->is_odom || !node_ptr->is_contour_match) return;
        geometry_msgs::Point nav_pos, vertex_pos;
        nav_pos = DPUtil::Point3DToGeoMsgPoint(node_ptr->position);
        vertex_pos = DPUtil::Point3DToGeoMsgPoint(node_ptr->ctnode->position);
        contour_align_marker.points.push_back(vertex_pos);
        contour_align_marker.points.push_back(nav_pos);
    };
    auto Draw_Edge = [&](const NavNodePtr& node_ptr) {
        geometry_msgs::Point p1, p2;
        p1 = DPUtil::Point3DToGeoMsgPoint(node_ptr->position);
        for (const auto& cnode : node_ptr->connect_nodes) {
            if (cnode == NULL) {
                ROS_WARN("Viz: node connect to a NULL node");
                continue;
            }
            // if (DPUtil::IsTypeInStack(cnode, node_ptr->contour_connects)) continue;
            // if (DPUtil::IsTypeInStack(cnode, node_ptr->trajectory_connects)) continue;
            p2 = DPUtil::Point3DToGeoMsgPoint(cnode->position);
            if (node_ptr->is_goal || cnode->is_goal) {
                goal_edge_marker.points.push_back(p1);
                goal_edge_marker.points.push_back(p2);
            } else if (node_ptr->is_odom || cnode->is_odom) {
                odom_edge_marker.points.push_back(p1);
                odom_edge_marker.points.push_back(p2);
            } else {
                // if (cnode->is_wall_insert || node_ptr->is_wall_insert) {
                edge_marker.points.push_back(p1);
                edge_marker.points.push_back(p2);                    
                // }                
            }
        }
        // contour edges
        for (const auto& ct_cnode : node_ptr->contour_connects) {
            if (ct_cnode == NULL) {
                ROS_WARN("Viz: node contour connect to a NULL node");
                continue;
            }
            p2 = DPUtil::Point3DToGeoMsgPoint(ct_cnode->position);
            contour_edge_marker.points.push_back(p1);
            contour_edge_marker.points.push_back(p2);
        }
        // top layer contour edges
        if (node_ptr->is_top_layer) {
            for (const auto& ct_cnode : node_ptr->contour_connects) {
                if (ct_cnode == NULL) {
                    ROS_WARN("Viz: node top layer contour connect to a NULL node");
                    continue;
                } else if (ct_cnode == node_ptr->down_node) continue;
                p2 = DPUtil::Point3DToGeoMsgPoint(ct_cnode->position);
                top_contour_marker.points.push_back(p1);
                top_contour_marker.points.push_back(p2);
            }
        }
        // bottom layer contour edges
        if (node_ptr->is_bottom_layer) {
            for (const auto& ct_cnode : node_ptr->contour_connects) {
                if (ct_cnode == NULL) {
                    ROS_WARN("Viz: node bottom layer contour connect to a NULL node");
                    continue;
                } else if (ct_cnode == node_ptr->up_node) continue;
                p2 = DPUtil::Point3DToGeoMsgPoint(ct_cnode->position);
                bottom_contour_marker.points.push_back(p1);
                bottom_contour_marker.points.push_back(p2);
            }
        }
        // inter navigation trajectory connections
        if (node_ptr->is_navpoint) {
            for (const auto& tj_cnode : node_ptr->trajectory_connects) {
                if (tj_cnode == NULL) {
                    ROS_WARN("Viz: node trajectory connect to a NULL node");
                    continue;
                }
                p2 = DPUtil::Point3DToGeoMsgPoint(tj_cnode->position);
                trajectory_edge_marker.points.push_back(p1);
                trajectory_edge_marker.points.push_back(p2);
            }
        }
    };
    auto Draw_Surf_Dir = [&](const NavNodePtr& node_ptr) {
        geometry_msgs::Point p1, p2, p3;
        p1 = DPUtil::Point3DToGeoMsgPoint(node_ptr->position);
        Point3D end_p;
        if (node_ptr->free_direct != NodeFreeDirect::PILLAR) {
            end_p = node_ptr->position + node_ptr->surf_dirs.first * DPUtil::kVizRatio;
            p2 = DPUtil::Point3DToGeoMsgPoint(end_p);
            corner_surf_marker.points.push_back(p1);
            corner_surf_marker.points.push_back(p2);
            corner_helper_marker.points.push_back(p2);
            end_p = node_ptr->position + node_ptr->surf_dirs.second * DPUtil::kVizRatio;
            p3 = DPUtil::Point3DToGeoMsgPoint(end_p);
            corner_surf_marker.points.push_back(p1);
            corner_surf_marker.points.push_back(p3);
            corner_helper_marker.points.push_back(p3);
        }
    };
    std::size_t idx = 0;
    const std::size_t graph_size = graph.size();
    nav_node_marker.points.resize(graph_size);
    for (const auto& nav_node_ptr : graph) {
        if (nav_node_ptr == NULL) {
            ROS_WARN("Viz: graph includes NULL nodes");
            continue;
        }
        const geometry_msgs::Point cpoint = DPUtil::Point3DToGeoMsgPoint(nav_node_ptr->position);
        nav_node_marker.points[idx] = cpoint;
        if (!nav_node_ptr->is_finalized) {
            unfinal_node_marker.points.push_back(cpoint);
        }
        // if (nav_node_ptr->up_node == NULL && nav_node_ptr->down_node != NULL && nav_node_ptr->free_direct == NodeFreeDirect::CONVEX) {
        //     top_node_marker.points.push_back(cpoint);
        // }
        if (nav_node_ptr->is_inserted || nav_node_ptr->is_wall_insert) {
            insert_node_marker.points.push_back(cpoint);
        }
        // if (nav_node_ptr->is_top_layer == true) {
        //     top_node_marker.points.push_back(cpoint);
        //     for (auto cnode : nav_node_ptr->contour_connects) {
        //         if (cnode->is_wall_insert && cnode->free_direct == NodeFreeDirect::INSERT) {
        //             const geometry_msgs::Point cpoint_tmp = DPUtil::Point3DToGeoMsgPoint(cnode->position);
        //             near_node_marker.points.push_back(cpoint_tmp);
        //             // cout<<"Node direct: "<<cnode->free_direct<<endl;
        //         }
        //     }
        // }
        if (nav_node_ptr->is_reach_goal) {
            reachable_node_marker.points.push_back(cpoint);
        }
        if (nav_node_ptr->is_navpoint) {
            internav_node_marker.points.push_back(cpoint);
        }
        if (nav_node_ptr->is_near_nodes) {
            near_node_marker.points.push_back(cpoint);
        }
        if (!nav_node_ptr->is_frontier) {
            covered_node_marker.points.push_back(cpoint);
        }
        Draw_Edge(nav_node_ptr);
        Draw_Surf_Dir(nav_node_ptr);
        Draw_Contour_Align(nav_node_ptr);
        idx ++;    
    } 
    nav_node_marker.points.resize(idx);
    graph_marker_array.markers.push_back(nav_node_marker);
    graph_marker_array.markers.push_back(unfinal_node_marker);
    graph_marker_array.markers.push_back(reachable_node_marker);
    graph_marker_array.markers.push_back(near_node_marker);
    graph_marker_array.markers.push_back(covered_node_marker);
    graph_marker_array.markers.push_back(internav_node_marker);
    graph_marker_array.markers.push_back(edge_marker);
    graph_marker_array.markers.push_back(goal_edge_marker);
    graph_marker_array.markers.push_back(contour_edge_marker);
    graph_marker_array.markers.push_back(odom_edge_marker);
    graph_marker_array.markers.push_back(trajectory_edge_marker);
    graph_marker_array.markers.push_back(corner_surf_marker);
    graph_marker_array.markers.push_back(corner_helper_marker);
    graph_marker_array.markers.push_back(contour_align_marker);
    graph_marker_array.markers.push_back(top_node_marker);
    graph_marker_array.markers.push_back(top_contour_marker);
    graph_marker_array.markers.push_back(bottom_contour_marker);
    graph_marker_array.markers.push_back(insert_node_marker);
    viz_graph_pub_.publish(graph_marker_array);
}

void DPVisualizer::VizMapGrids(const PointStack& neighbor_centers, const PointStack& occupancy_centers,
                               const float& ceil_length, const float& ceil_height)
{
    MarkerArray map_grid_marker_array;
    Marker neighbor_marker, occupancy_marker;
    neighbor_marker.type = Marker::CUBE_LIST;
    occupancy_marker.type = Marker::CUBE_LIST;
    this->SetMarker(VizColor::GREEN, "neighbor_grids", ceil_length / DPUtil::kVizRatio, 0.25f, neighbor_marker);
    this->SetMarker(VizColor::RED, "occupancy_grids", ceil_length / DPUtil::kVizRatio, 0.25f, occupancy_marker);
    neighbor_marker.scale.z = occupancy_marker.scale.z = ceil_height;
    const std::size_t N1 = neighbor_centers.size();
    const std::size_t N2 = occupancy_centers.size();
    neighbor_marker.points.resize(N1), occupancy_marker.points.resize(N2);
    for (std::size_t i=0; i<N1; i++) {
        geometry_msgs::Point p = DPUtil::Point3DToGeoMsgPoint(neighbor_centers[i]);
        neighbor_marker.points[i] = p;
    }
    for (std::size_t i=0; i<N2; i++) {
        geometry_msgs::Point p = DPUtil::Point3DToGeoMsgPoint(occupancy_centers[i]);
        occupancy_marker.points[i] = p;
    }
    map_grid_marker_array.markers.push_back(neighbor_marker);
    map_grid_marker_array.markers.push_back(occupancy_marker);
    viz_map_pub_.publish(map_grid_marker_array);
}

void DPVisualizer::SetMarker(const VizColor& color, 
                             const std::string& ns,
                             const float& scale,
                             const float& alpha,  
                             Marker& scan_marker, 
                             const float& scale_ratio) 
{
    scan_marker.header.frame_id = DPUtil::worldFrameId;
    scan_marker.header.stamp = ros::Time::now();
    scan_marker.id = 0;
    scan_marker.ns = ns;
    scan_marker.action = Marker::ADD;
    scan_marker.scale.x = scan_marker.scale.y = scan_marker.scale.z = scale * scale_ratio;
    scan_marker.pose.orientation.x = 0.0;
    scan_marker.pose.orientation.y = 0.0;
    scan_marker.pose.orientation.z = 0.0;
    scan_marker.pose.orientation.w = 1.0;
    scan_marker.pose.position.x = 0.0;
    scan_marker.pose.position.y = 0.0;
    scan_marker.pose.position.z = 0.0;
    DPVisualizer::SetColor(color, alpha, scan_marker);
}

void DPVisualizer::VizPointCloud(const ros::Publisher& viz_pub, 
                                 const PointCloudPtr& pc) 
{
    sensor_msgs::PointCloud2 msg_pc;
    pcl::toROSMsg(*pc, msg_pc);
    msg_pc.header.frame_id = DPUtil::worldFrameId;
    msg_pc.header.stamp = ros::Time::now();
    viz_pub.publish(msg_pc);
}

void DPVisualizer::SetColor(const VizColor& color, 
                            const float& alpha, 
                            Marker& scan_marker)
{
    std_msgs::ColorRGBA c;
    c.a = alpha;
    if (color == VizColor::RED) {
    c.r = 0.9f, c.g = c.b = 0.f;
    }
    else if (color == VizColor::ORANGE) {
    c.r = 1.0f, c.g = 0.45f, c.b = 0.1f;
    }
    else if (color == VizColor::BLACK) {
    c.r = c.g = c.b = 0.1f;
    }
    else if (color == VizColor::YELLOW) {
    c.r = c.g = 0.9f, c.b = 0.1;
    }
    else if (color == VizColor::BLUE) {
    c.b = 1.0f, c.r = 0.1f, c.g = 0.1f;
    }
    else if (color == VizColor::GREEN) {
    c.g = 0.9f, c.r = c.b = 0.f;
    }
    else if (color == VizColor::EMERALD) {
    c.g = c.b = 0.9f, c.r = 0.f;
    }
    else if (color == VizColor::WHITE) {
    c.r = c.g = c.b = 0.9f;
    }
    else if (color == VizColor::MAGNA) {
    c.r = c.b = 0.9f, c.g = 0.f;
    }
    else if (color == VizColor::PURPLE) {
    c.r = c.b = 0.5f, c.g = 0.f;
    }
    scan_marker.color = c;
}

