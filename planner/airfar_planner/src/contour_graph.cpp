/*
 * Dynamic Route Planner
 * Copyright (C) 2021 Fan Yang - All rights reserved
 * fanyang2@andrew.cmu.edu,   
 */



#include "airfar_planner/contour_graph.h"
#include "airfar_planner/intersection.h"

/***************************************************************************************/

void ContourGraph::Init(const ContourGraphParams& params) {
    ctgraph_params_ = params;
    ContourGraph::multi_contour_graph_.clear();
    ContourGraph::multi_contour_polygons_.clear();
    ContourGraph::multi_global_contour_.clear();
    ContourGraph::multi_contour_graph_kdtree_.clear();
    ALIGN_ANGLE_COS = cos(M_PI - DPUtil::kAcceptAlign);
    const int N_Layers = DPUtil::layerIdx2Height_.size();
    ContourGraph::multi_contour_graph_.resize(N_Layers);
    ContourGraph::multi_contour_polygons_.resize(N_Layers);
    ContourGraph::multi_global_contour_.resize(N_Layers);
    ContourGraph::multi_contour_graph_kdtree_.resize(N_Layers);

    indexParams.reset(new cv::flann::KDTreeIndexParams(ctgraph_params_.KD_TREE_K));
}

void ContourGraph::UpdateContourGraph(const NavNodePtr& odom_node_ptr,
                                      const int& layer_idx,
                                      const std::vector<std::vector<Point3D>>& filtered_contours) {
    if (filtered_contours.empty()) return;
    // ROS_WARN("odom node layer id: %d", odom_node_ptr->layer_id);
    // ROS_WARN("odom position: %f, %f, %f", odom_node_ptr->position.x, odom_node_ptr->position.y, odom_node_ptr->position.z);
    odom_node_ptr_ = odom_node_ptr;
    this->ClearContourGraph(layer_idx);
    for (const auto& poly : filtered_contours) {
        PolygonPtr new_poly_ptr = NULL;
        this->CreatePolygon(poly, layer_idx, new_poly_ptr);
        this->AddPolyToContourPolygon(new_poly_ptr, layer_idx);
    }
    if (layer_idx == odom_node_ptr_->layer_id) {
        ContourGraph::UpdateOdomFreePosition(odom_node_ptr_, DPUtil::free_odom_p);
    }
    for (const auto& poly_ptr : ContourGraph::multi_contour_polygons_[layer_idx]) {
        CTNodePtr new_ctnode_ptr = NULL;
        if (poly_ptr->is_pillar) {
            Point3D mean_p = DPUtil::AveragePoints(poly_ptr->vertices);
            this->CreateCTNode(mean_p, layer_idx, new_ctnode_ptr, poly_ptr, true);
            this->AddCTNodeToGraph(new_ctnode_ptr, layer_idx);
        } else {
            CTNodeStack ctnode_stack;
            ctnode_stack.clear();
            const int N = poly_ptr->vertices.size();
            for (std::size_t idx=0; idx<N; idx++) { 
                this->CreateCTNode(poly_ptr->vertices[idx], layer_idx, new_ctnode_ptr, poly_ptr, false);
                ctnode_stack.push_back(new_ctnode_ptr);
            }
            // add connections to contour nodes
            for (int idx=0; idx<N; idx++) {
                int ref_idx = DPUtil::Mod(idx-1, N);
                ctnode_stack[idx]->front = ctnode_stack[ref_idx];
                ref_idx = DPUtil::Mod(idx+1, N);
                ctnode_stack[idx]->back = ctnode_stack[ref_idx];

                // check if the contour node is a wall corner
                if (!ctnode_stack[idx]->is_wall_corner) {
                    if ((ctnode_stack[idx]->front->position - ctnode_stack[idx]->position).norm() > DPUtil::kSensorRange*ctgraph_params_.wall_insert_factor &&
                    (ctnode_stack[idx]->position - odom_node_ptr_->position).norm() < (ctnode_stack[idx]->front->position - odom_node_ptr_->position).norm()-20.0*ctgraph_params_.voxel_dim) {
                        SetWallCornerNodes(ctnode_stack[idx], ctnode_stack[idx]->front, layer_idx);
                    }
                    if ((ctnode_stack[idx]->back->position - ctnode_stack[idx]->position).norm() > DPUtil::kSensorRange*ctgraph_params_.wall_insert_factor &&
                    (ctnode_stack[idx]->position - odom_node_ptr_->position).norm() < (ctnode_stack[idx]->back->position - odom_node_ptr_->position).norm()-20.0*ctgraph_params_.voxel_dim) {
                        SetWallCornerNodes(ctnode_stack[idx], ctnode_stack[idx]->back, layer_idx);
                    }
                }
                this->AddCTNodeToGraph(ctnode_stack[idx], layer_idx);
            }
        }
    }
    this->BuildKDTreeOnContourGraph(ContourGraph::multi_contour_graph_[layer_idx], layer_idx);
    this->AnalysisSurfAngleAndConvexity(ContourGraph::multi_contour_graph_[layer_idx]);
}

void ContourGraph::MatchContourWithNavGraph(const NodePtrStack& nav_graph, 
                                            const std::vector<int>& cur_layer_idxs, 
                                            CTNodeStack& new_convex_vertices) 
/* Match current contour with global navigation nodes */
{
    for (const auto& node_ptr : nav_graph) {
        node_ptr->is_contour_match = false;
        node_ptr->ctnode = NULL;
    }
    new_convex_vertices.clear();
    for (const int& layer_id : cur_layer_idxs) {
        for (const auto& ctnode_ptr : ContourGraph::multi_contour_graph_[layer_id]) {
            ctnode_ptr->is_global_match = false;
            ctnode_ptr->nav_node_id = 0; // note: 0 is the id of odom node 
            if (ctnode_ptr->free_direct != NodeFreeDirect::UNKNOW) {
                const NavNodePtr matched_node = this->NearestNavNodeForCTNode(ctnode_ptr, nav_graph, layer_id);                     

                // if (ctnode_ptr->is_wall_insert && matched_node != NULL)
                //     cout<<"CG: WALL CONTOUR MATCHED"<< endl;
                // else if (ctnode_ptr->is_wall_insert && matched_node == NULL)
                //     cout<<"CG: WALL CONTOUR NULL"<< endl;

                if (matched_node != NULL) {
                    ctnode_ptr->is_global_match = true;
                    ctnode_ptr->nav_node_id = matched_node->id;
                    ctnode_ptr->poly_ptr->is_connect = true;
                    matched_node->ctnode = ctnode_ptr;
                    matched_node->is_contour_match = true;
                }
            }
      
        }
        for (const auto& ctnode_ptr : ContourGraph::multi_contour_graph_[layer_id]) {
            if (!ctnode_ptr->is_global_match && ctnode_ptr->free_direct != NodeFreeDirect::UNKNOW) {
                // check wall contour
                if (ctnode_ptr->free_direct != NodeFreeDirect::PILLAR && !ctnode_ptr->is_wall_insert) {
                    const float dot_value = ctnode_ptr->surf_dirs.first * ctnode_ptr->surf_dirs.second;
                    if (dot_value < ALIGN_ANGLE_COS) continue; // wall detected
                }
                new_convex_vertices.push_back(ctnode_ptr);
            }
        }
    }
}

bool ContourGraph::IsNavNodesConnectFreePolygon(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2, const bool& is_local_only, const bool& layer_limited) {
    if (node_ptr1->is_navpoint || node_ptr2->is_navpoint) {
        if (node_ptr1->layer_id == node_ptr2->layer_id && (node_ptr1->position - node_ptr2->position).norm() < DPUtil::kNavClearDist) { // connect to internav node
            return true;
        }
    }
    bool is_layer_limit = layer_limited;
    if (node_ptr1->is_odom || node_ptr2->is_odom) is_layer_limit = false;
    ConnectPair cedge;
    int start_layer, end_layer;
    if (node_ptr1->layer_id <= node_ptr2->layer_id) {
        cedge = ContourGraph::ReprojectEdge(node_ptr1, node_ptr2, DPUtil::kNavClearDist, false);
        start_layer = node_ptr1->layer_id, end_layer = node_ptr2->layer_id;
    } else {
        cedge = ContourGraph::ReprojectEdge(node_ptr2, node_ptr1, DPUtil::kNavClearDist, false);
        start_layer = node_ptr2->layer_id, end_layer = node_ptr1->layer_id;
    }
    bool is_global_check = false;
    if (!is_local_only && (!node_ptr1->is_contour_match || !node_ptr2->is_contour_match) && 
        ContourGraph::IsNeedGlobalCheck(node_ptr1->position, node_ptr2->position, DPUtil::odom_pos)) 
    {
        is_global_check = true;
    }
    

    return ContourGraph::IsPointsConnectFreePolygon(cedge, start_layer, end_layer, is_global_check, is_layer_limit);
}

bool ContourGraph::IsNavToOdomConnectFreePolygon(const NavNodePtr& node_ptr, const NavNodePtr& odom_ptr) {
    // ros::Time start_time = ros::Time::now();
    if ((node_ptr->position - odom_ptr->position).norm() < DPUtil::kNavClearDist) return true;
    ConnectPair cedge;
    ConnectPair cedge2;
    int start_layer, end_layer;
    auto diff = odom_ptr->position - node_ptr->position;
    // auto diff = node_ptr->position - odom_ptr->position;
    diff = diff/ diff.norm();
    cv::Point2f diff2d;
    diff2d.x = diff.x;
    diff2d.y = diff.y;
    diff2d = diff2d * 0.4;

    // if (node_ptr->layer_id <= odom_ptr->layer_id) {
    //     cedge = ContourGraph::ReprojectEdge(node_ptr, odom_ptr, DPUtil::kNavClearDist);
    //     cedge.start_p = cedge.start_p + diff2d;
    //     start_layer = node_ptr->layer_id, end_layer = odom_ptr->layer_id;
    // } else {
    //     cedge = ContourGraph::ReprojectEdge(odom_ptr, node_ptr, DPUtil::kNavClearDist);
    //     cedge.end_p = cedge.end_p + diff2d;
    //     start_layer = odom_ptr->layer_id, end_layer = node_ptr->layer_id;
    // }

    if (node_ptr->layer_id <= odom_ptr->layer_id) {
        cedge = ContourGraph::ReprojectEdge(node_ptr, odom_ptr, DPUtil::kNavClearDist, false);
        cedge2 = ContourGraph::ReprojectEdge(node_ptr, odom_ptr, DPUtil::kNavClearDist, true);
        cedge.start_p = cedge.start_p + diff2d;
        cedge2.start_p = cedge2.start_p;

        start_layer = node_ptr->layer_id, end_layer = odom_ptr->layer_id;
    } else {
        cedge = ContourGraph::ReprojectEdge(odom_ptr, node_ptr, DPUtil::kNavClearDist, false);
        cedge2 = ContourGraph::ReprojectEdge(odom_ptr, node_ptr, DPUtil::kNavClearDist, true);
        cedge.end_p = cedge.end_p + diff2d;
        cedge2.end_p = cedge.end_p;

        start_layer = odom_ptr->layer_id, end_layer = node_ptr->layer_id;
    }

    const bool is_global_check = ContourGraph::IsNeedGlobalCheck(node_ptr->position, odom_ptr->position, DPUtil::odom_pos);
    // return (ContourGraph::IsPointsConnectFreePolygon(cedge, start_layer, end_layer, is_global_check, false) || 
    //        ContourGraph::IsPointsConnectFreePolygon(cedge2, start_layer, end_layer, is_global_check, false));

    // bool res1 = ContourGraph::IsPointsConnectFreePolygon(cedge, start_layer, end_layer, is_global_check, false);
    // ROS_WARN("IsNavToOdomConnectFreePolygon time: %f", (ros::Time::now()-start_time).toSec());


    if (node_ptr->is_inserted) {
        // print cedge use roswarn
        // ROS_ERROR("start_p: %f, %f", cedge.start_p.x, cedge.start_p.y);
        // ROS_ERROR("end_p: %f, %f", cedge.end_p.x, cedge.end_p.y);
        // ROS_WARN("==============================");
        // ROS_WARN("start_p2: %f, %f", cedge2.start_p.x, cedge2.start_p.y);
        // ROS_WARN("end_p2: %f, %f", cedge2.end_p.x, cedge2.end_p.y);
        // print diff2d
        // ROS_WARN("diff2d: %f, %f", diff2d.x, diff2d.y);
        return ContourGraph::IsPointsConnectFreePolygon(cedge, start_layer, end_layer, is_global_check, false);
        // return (ContourGraph::IsPointsConnectFreePolygonTest(cedge, start_layer, end_layer, is_global_check, false) || 
        // ContourGraph::IsPointsConnectFreePolygonTest(cedge2, start_layer, end_layer, is_global_check, false));
    } else {
        return ContourGraph::IsPointsConnectFreePolygon(cedge, start_layer, end_layer, is_global_check, false);
    }


    return ContourGraph::IsPointsConnectFreePolygon(cedge, start_layer, end_layer, is_global_check, false);
}

bool ContourGraph::IsNavToGoalConnectFreePolygon(const NavNodePtr& node_ptr, const NavNodePtr& goal_ptr) {
    if ((node_ptr->position - goal_ptr->position).norm() < DPUtil::kNavClearDist) return true;
    ConnectPair cedge, cedge2;
    int start_layer, end_layer;

    auto diff = goal_ptr->position - node_ptr->position;
    diff = diff/ diff.norm();
    cv::Point2f diff2d;
    diff2d.x = diff.x;
    diff2d.y = diff.y;
    diff2d = diff2d * 0.5;

    if (node_ptr->layer_id <= goal_ptr->layer_id) {
        cedge = ContourGraph::ReprojectEdge(node_ptr, goal_ptr, DPUtil::kNavClearDist, false);
        // cedge2 = ContourGraph::ReprojectEdge(node_ptr, goal_ptr, DPUtil::kNavClearDist, true);
        cedge.start_p = cedge.start_p + diff2d;
        // cedge2.start_p = cedge2.start_p + diff2d;
        start_layer = node_ptr->layer_id, end_layer = goal_ptr->layer_id;
    } else {
        cedge = ContourGraph::ReprojectEdge(goal_ptr, node_ptr, DPUtil::kNavClearDist, false);
        // cedge2 = ContourGraph::ReprojectEdge(goal_ptr, node_ptr, DPUtil::kNavClearDist, true);
        cedge.end_p = cedge.end_p + diff2d;
        // cedge2.end_p = cedge2.end_p + diff2d;
        start_layer = goal_ptr->layer_id, end_layer = node_ptr->layer_id;
    }

    const bool is_global_check = ContourGraph::IsNeedGlobalCheck(node_ptr->position, goal_ptr->position, DPUtil::odom_pos);
    return ContourGraph::IsPointsConnectFreePolygon(cedge, start_layer, end_layer, is_global_check, false);
}

bool ContourGraph::IsPointsConnectFreePolygon(const ConnectPair& cedge,
                                              const int& start_layer,
                                              const int& end_layer,
                                              const bool& is_global_check,
                                              const bool& layer_limited)
{
    if (layer_limited && (end_layer - start_layer > DPUtil::kNeighborLayers)) return false; // max acrossing layers

    cv::Point2f unit_diff = cedge.end_p - cedge.start_p;
    if (start_layer < end_layer) {
        unit_diff /= (float)((end_layer - start_layer) * 2);
    }
    cv::Point2f start_cv_p = cedge.start_p;
    for (int layer_id=start_layer; layer_id<=end_layer; layer_id++) {
        const int segs = (layer_id == start_layer || layer_id == end_layer) ? 1 : 2;
        const ConnectPair check_edge(start_cv_p, start_cv_p + unit_diff * segs); 

        if (is_global_check) {
            for (const auto& contour : ContourGraph::multi_global_contour_[layer_id]) {
                if ((contour.first - DPUtil::odom_pos).norm() > DPUtil::kSensorRange || 
                    (contour.second - DPUtil::odom_pos).norm() > DPUtil::kSensorRange) 
                {
                    if (ContourGraph::IsEdgeCollideSegment(contour, check_edge)) {
                        return false;
                    }
                }
            }
        }

        if (ContourGraph::multi_contour_polygons_[layer_id].empty()) continue;
        for (const auto& poly_ptr : ContourGraph::multi_contour_polygons_[layer_id]) {
            if (poly_ptr->is_pillar) continue;
            if (ContourGraph::IsEdgeCollidePoly(poly_ptr->vertices, check_edge)) {
                return false;
            }
        }
        start_cv_p = start_cv_p + unit_diff * segs;
    }
    return true;
}

bool ContourGraph::IsPointsConnectFreePolygonTest(const ConnectPair& cedge,
                                                const int& start_layer,
                                                const int& end_layer,
                                                const bool& is_global_check,
                                                const bool& layer_limited)
{
    if (layer_limited && (end_layer - start_layer > DPUtil::kNeighborLayers)) return false; // max acrossing layers

    cv::Point2f unit_diff = cedge.end_p - cedge.start_p;
    if (start_layer < end_layer) {
        unit_diff /= (float)((end_layer - start_layer) * 2);
    }

    cv::Point2f start_cv_p = cedge.start_p;
    for (int layer_id=start_layer; layer_id<=end_layer; layer_id++) {
        const int segs = (layer_id == start_layer || layer_id == end_layer) ? 1 : 2;
        const ConnectPair check_edge(start_cv_p, start_cv_p + unit_diff * segs); 
        // ROS_WARN("Checkpoint 1");
        if (is_global_check) {
            for (const auto& contour : ContourGraph::multi_global_contour_[layer_id]) {
                if ((contour.first - DPUtil::odom_pos).norm() > DPUtil::kSensorRange || 
                    (contour.second - DPUtil::odom_pos).norm() > DPUtil::kSensorRange) 
                {
                    if (ContourGraph::IsEdgeCollideSegment(contour, check_edge)) {
                        return false;
                    }
                }
            }
        }
        // ROS_WARN("Checkpoint 2");
        if (ContourGraph::multi_contour_polygons_[layer_id].empty()) continue;
        for (const auto& poly_ptr : ContourGraph::multi_contour_polygons_[layer_id]) {
            if (poly_ptr->is_pillar) continue;
            ROS_WARN("Checkpoint 3: %d", ContourGraph::IsEdgeCollidePoly(poly_ptr->vertices, check_edge)); 
            if (ContourGraph::IsEdgeCollidePoly(poly_ptr->vertices, check_edge)) {

                // // DEBUG
                const int N = poly_ptr->vertices.size();
                for (int i=0; i<N; i++) {
                    const PointPair line(poly_ptr->vertices[i], poly_ptr->vertices[DPUtil::Mod(i+1, N)]);
                    if (ContourGraph::IsEdgeCollideSegment(line, check_edge)) {
                        // print line and edge
                        ROS_WARN("line start: %f, %f", line.first.x, line.first.y);
                        ROS_WARN("line end: %f, %f", line.second.x, line.second.y);
                        ROS_WARN("edge start: %f, %f", check_edge.start_p.x, check_edge.start_p.y);
                        ROS_WARN("edge end: %f, %f", check_edge.end_p.x, check_edge.end_p.y);
                    }
                }



                return false;
            }
        }
        // ROS_WARN("Checkpoint 4");        
        start_cv_p = start_cv_p + unit_diff * segs;
    }

    return true;
}

bool ContourGraph::IsNavNodesConnectFromContour(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
    if (node_ptr1->is_odom || node_ptr2->is_odom) return false;
    if (!node_ptr1->is_contour_match || !node_ptr2->is_contour_match) {
        return false;
    }
    const CTNodePtr ctnode1 = node_ptr1->ctnode;
    const CTNodePtr ctnode2 = node_ptr2->ctnode;
    if (ctnode1 == ctnode2) return false;
    return ContourGraph::IsCTNodesConnectFromContour(ctnode1, ctnode2);
}

bool ContourGraph::IsCTNodesConnectFromContour(const CTNodePtr& ctnode1, const CTNodePtr& ctnode2) {
    if (ctnode1->up == ctnode2 || ctnode1->down == ctnode2) return true;
    if (ctnode2->up == ctnode1 || ctnode2->down == ctnode1) return true;
    if (ctnode1 == ctnode2 || ctnode1->poly_ptr != ctnode2->poly_ptr) return false;
    // forward search
    CTNodePtr next_ctnode = ctnode1->front; 
    CTNodePtr last_ctnode = ctnode1;
    while (next_ctnode != NULL && next_ctnode != ctnode1) {
        if (next_ctnode == ctnode2) {
            return true;
        }
        if (next_ctnode->is_global_match || DPUtil::VerticalDistToLine(ctnode1->position, ctnode2->position, next_ctnode->position) > DPUtil::kNearDist) 
        {
            break;
        } else {
            CTNodePtr tmp_ctnode = next_ctnode->front;
            if (tmp_ctnode == last_ctnode) {
                next_ctnode = next_ctnode->back;
            } else {
                next_ctnode = next_ctnode->front;                
            }
        }
    }
    // backward search
    next_ctnode = ctnode1->back;
    while (next_ctnode != NULL && next_ctnode != ctnode1) {
        if (next_ctnode == ctnode2) {
            return true;
        }
        if (next_ctnode->is_global_match || DPUtil::VerticalDistToLine(ctnode1->position, ctnode2->position, next_ctnode->position) > DPUtil::kNearDist) 
        {
            break;
        } else {
            CTNodePtr tmp_ctnode = next_ctnode->back;
            if (tmp_ctnode == last_ctnode) {
                next_ctnode = next_ctnode->front;
            } else {
                next_ctnode = next_ctnode->back;                
            }
        }
    }
    return false;
}

void ContourGraph::CreateCTNode(const Point3D& pos, const int& layer_id, CTNodePtr& ctnode_ptr, const PolygonPtr& poly_ptr, const bool& is_pillar) {
    ctnode_ptr = std::make_shared<CTNode>();
    ctnode_ptr->layer_id = layer_id;
    ctnode_ptr->position = pos;
    ctnode_ptr->front = NULL;
    ctnode_ptr->back  = NULL;
    ctnode_ptr->is_global_match = false;
    ctnode_ptr->is_wall_corner = false;
    ctnode_ptr->is_wall_insert = false;
    ctnode_ptr->nav_node_id = 0;
    ctnode_ptr->poly_ptr = poly_ptr;
    ctnode_ptr->free_direct = is_pillar ? NodeFreeDirect::PILLAR : NodeFreeDirect::UNKNOW;
    ctnode_ptr->connect_nodes.clear();
}

void ContourGraph::CreatePolygon(const PointStack& poly_points, const int& layer_id, PolygonPtr& poly_ptr) {
    poly_ptr = std::make_shared<Polygon>();
    poly_ptr->layer_id = layer_id;
    poly_ptr->N = poly_points.size();
    poly_ptr->vertices = poly_points;
    poly_ptr->is_connect = false;
    poly_ptr->is_visiable = false;
    poly_ptr->is_robot_inside = DPUtil::PointInsideAPoly(poly_points, odom_node_ptr_->position);
    poly_ptr->is_pillar = this->IsAPillarPolygon(poly_points);
}

NavNodePtr ContourGraph::NearestNavNodeForCTNode(const CTNodePtr& ctnode_ptr, const NodePtrStack& nav_graph, const int& layer_id) {
    // ROS_WARN("NearestNavNodeForCTNode");
    float nearest_dist = DPUtil::kINF;
    NavNodePtr nearest_node = NULL;
    float min_edist = DPUtil::kINF;
    for (const auto& node_ptr : nav_graph) {
        if (node_ptr->layer_id != layer_id || node_ptr->is_odom || node_ptr->is_navpoint || node_ptr->is_goal) continue;
        Point3D nav_node_pos = node_ptr->position;
        nav_node_pos.z = ctnode_ptr->position.z;
        if (ctnode_ptr->free_direct != NodeFreeDirect::PILLAR && node_ptr->free_direct != NodeFreeDirect::UNKNOW && node_ptr->free_direct != NodeFreeDirect::PILLAR) {
            const Point3D topo_dir1 = DPUtil::SurfTopoDirect(node_ptr->surf_dirs);
            const Point3D topo_dir2 = DPUtil::SurfTopoDirect(ctnode_ptr->surf_dirs);
            if (topo_dir1 * topo_dir2 < 0.0) {
                continue;
            } 
        }
        const float edist = (nav_node_pos - ctnode_ptr->position).norm();
        if (edist < DPUtil::kNearDist && edist < min_edist) {
            nearest_node = node_ptr;
            min_edist = edist;
        }
    }
    if (nearest_node != NULL && nearest_node->is_contour_match) {
        Point3D nav_node_pos = nearest_node->position;
        nav_node_pos.z = nearest_node->ctnode->position.z; 
        const float pre_dist = (nav_node_pos - nearest_node->ctnode->position).norm();
        if (min_edist < pre_dist) {
            // reset matching for previous ctnode
            nearest_node->ctnode->is_global_match = false;
            nearest_node->ctnode->nav_node_id = 0;
        } else {
            nearest_node = NULL;
        }
    }
    return nearest_node;
}

void ContourGraph::AnalysisSurfAngleAndConvexity(const CTNodeStack& contour_graph) {
    for (const auto& ctnode_ptr : contour_graph) {
        if (ctnode_ptr->free_direct == NodeFreeDirect::PILLAR || ctnode_ptr->poly_ptr->is_pillar) {
            ctnode_ptr->surf_dirs = {Point3D(0,0,-1), Point3D(0,0,-1)};
            ctnode_ptr->free_direct = NodeFreeDirect::PILLAR;
        } else {
            CTNodePtr next_ctnode;
            // front direction
            next_ctnode = ctnode_ptr->front;
            Point3D start_p = ctnode_ptr->position;
            Point3D end_p = next_ctnode->position;
            float edist = (end_p - ctnode_ptr->position).norm();
            while (next_ctnode != NULL && next_ctnode != ctnode_ptr && edist < ctgraph_params_.kAroundDist) {
                next_ctnode = next_ctnode->front;
                start_p = end_p;
                end_p = next_ctnode->position;
                edist = (end_p - ctnode_ptr->position).norm();
            }
            if (edist < ctgraph_params_.kAroundDist) {
                ROS_DEBUG("CG: This Node should be a pillar.");
                ctnode_ptr->surf_dirs = {Point3D(0,0,-1), Point3D(0,0,-1)}; // TODO!
                ctnode_ptr->poly_ptr->is_pillar = true;
                ctnode_ptr->free_direct = NodeFreeDirect::PILLAR;
                continue;
            } else {
                ctnode_ptr->surf_dirs.first = DPUtil::ContourSurfDirs(end_p, start_p, ctnode_ptr->position, ctgraph_params_.kAroundDist);
            }
            // back direction
            next_ctnode = ctnode_ptr->back;
            start_p = ctnode_ptr->position;
            end_p   = next_ctnode->position;
            edist = (end_p - ctnode_ptr->position).norm();
            while (next_ctnode != NULL && next_ctnode != ctnode_ptr && edist < ctgraph_params_.kAroundDist) {
                next_ctnode = next_ctnode->back;
                start_p = end_p;
                end_p = next_ctnode->position;
                edist = (end_p - ctnode_ptr->position).norm();
            }
            if (edist < ctgraph_params_.kAroundDist) {
                ROS_DEBUG("CG: This Node should be a pillar.");
                ctnode_ptr->surf_dirs = {Point3D(0,0,-1), Point3D(0,0,-1)}; // TODO!
                ctnode_ptr->poly_ptr->is_pillar = true;
                ctnode_ptr->free_direct = NodeFreeDirect::PILLAR;
                continue;
            } else {
                ctnode_ptr->surf_dirs.second = DPUtil::ContourSurfDirs(end_p, start_p, ctnode_ptr->position, ctgraph_params_.kAroundDist);
            }
        }
        // analysis convexity (except pillar)
        this->AnalysisConvexityOfCTNode(ctnode_ptr);
    }
}

bool ContourGraph::IsAPillarPolygon(const PointStack& vertex_points) {
    if (vertex_points.size() < 3) return true;
    float sum_pixel_dist = 0;
    Point3D prev_p(vertex_points[0]);
    for (std::size_t i=1; i<vertex_points.size(); i++) {
        const Point3D cur_p(vertex_points[i]);
        const float dist = std::hypotf(cur_p.x - prev_p.x, cur_p.y - prev_p.y);
        sum_pixel_dist += dist;
        if (sum_pixel_dist > ctgraph_params_.kPillarPerimeter) {
            return false;
        }
        prev_p = cur_p;
    }
    return true;
}

bool ContourGraph::IsEdgeCollideSegment(const PointPair& line, const ConnectPair& edge) {
    const cv::Point2f start_p(line.first.x, line.first.y);
    const cv::Point2f end_p(line.second.x, line.second.y);

    // cv::Point2f vec1 = end_p - start_p;
    // cv::Point2f vec2 = edge.end_p - edge.start_p;
    // vec1 = vec1 / cv::norm(vec1);
    // vec2 = vec2 / cv::norm(vec2);
    // float cos_value = vec1.x * vec2.x + vec1.y * vec2.y;
    // if (cos_value > this->ALIGN_ANGLE_COS) return false;
    if (POLYOPS::doIntersect(start_p, end_p, edge.start_p, edge.end_p)) {
        return true;
    }
    return false;
}

cv::Point2f ContourGraph::getIntersectionPoint(const PointPair& line, const ConnectPair& edge) {
    const cv::Point2f start_p(line.first.x, line.first.y);
    const cv::Point2f end_p(line.second.x, line.second.y);
    const cv::Point2f inter_p = POLYOPS::IntersectionwithLine(start_p, end_p, edge.start_p, edge.end_p);
    // if (POLYOPS::onSegment(edge.start_p, inter_p, edge.end_p)) {
    //     return inter_p;
    // }
    return inter_p;
}

bool ContourGraph::IsEdgeCollidePoly(const PointStack& poly, const ConnectPair& edge) {
    const int N = poly.size();
    if (N < 3) cout<<"Poly vertex size less than 3."<<endl;
    for (int i=0; i<N; i++) {
        const PointPair line(poly[i], poly[DPUtil::Mod(i+1, N)]);
        if (ContourGraph::IsEdgeCollideSegment(line, edge)) {
            return true;
        }
    }
    return false;
}

void ContourGraph::AnalysisConvexityOfCTNode(const CTNodePtr& ctnode_ptr) 
{
    // ROS_WARN("AnalysisConvexityOfCTNode");
    if (ctnode_ptr->free_direct == NodeFreeDirect::PILLAR || ctnode_ptr->free_direct == NodeFreeDirect::INSERT) return;
    if (ctnode_ptr->surf_dirs.first == Point3D(0,0,-1) || 
        ctnode_ptr->surf_dirs.second == Point3D(0,0,-1) || 
        ctnode_ptr->poly_ptr->is_pillar) 
    {
        ctnode_ptr->surf_dirs.first = Point3D(0,0,-1), ctnode_ptr->surf_dirs.second == Point3D(0,0,-1);
        ctnode_ptr->free_direct = NodeFreeDirect::PILLAR;
        return;
    }
    bool is_wall = false;
    const Point3D topo_dir = DPUtil::SurfTopoDirect(ctnode_ptr->surf_dirs, is_wall);
    if (is_wall && !ctnode_ptr->is_wall_insert) {
        ctnode_ptr->free_direct = NodeFreeDirect::UNKNOW;
        return;
    }
    const Point3D ev_p = ctnode_ptr->position + topo_dir * DPUtil::kLeafSize;
    if (DPUtil::IsConvexPointFromFree(ctnode_ptr->poly_ptr->vertices, ev_p, DPUtil::free_odom_p)) {
        ctnode_ptr->free_direct = NodeFreeDirect::CONVEX;
    } else {
        ctnode_ptr->free_direct = NodeFreeDirect::CONCAVE;
    }
}

void ContourGraph::ExtractGlobalContours(const NodePtrStack& nav_graph, const std::vector<int>& cur_layers) {
    for (const int& layer_id : cur_layers) {
        ContourGraph::multi_global_contour_[layer_id].clear();
    }
    const int N = nav_graph.size();
    for (std::size_t i=0; i<N; i++) {
        if (!DPUtil::IsTypeInStack(nav_graph[i]->layer_id, cur_layers)) continue;
        for (std::size_t j=0; j<N; j++) {
            if (i == j || j > i || !DPUtil::IsTypeInStack(nav_graph[j]->layer_id, cur_layers)) continue;
            if (DPUtil::IsTypeInStack(nav_graph[j], nav_graph[i]->contour_connects)) {
                const int layer_id = nav_graph[i]->layer_id;
                ContourGraph::multi_global_contour_[layer_id].push_back({nav_graph[i]->position, nav_graph[j]->position});
            }
        } 
    }
}

void ContourGraph::UpdateOdomFreePosition(const NavNodePtr& odom_ptr, Point3D& global_free_p) {
    Point3D free_p = odom_ptr->position;
    bool is_free_p = true;
    PointStack free_sample_points;
    for (const auto& poly_ptr : ContourGraph::multi_contour_polygons_[odom_ptr->layer_id]) {
        if (!poly_ptr->is_pillar && poly_ptr->is_robot_inside) {
            is_free_p = false;
            const Point3D avg_p = (free_p + global_free_p) / 2.0;
            DPUtil::CreatePointsAroundCenter(avg_p, DPUtil::kNearDist, DPUtil::kLeafSize*2.0, free_sample_points);
            break;
        }
    }
    if (is_free_p) {
        global_free_p = free_p;
    } else {
        for (const auto& p : free_sample_points) {
            bool is_sample_free = true;
            for (const auto& poly_ptr : ContourGraph::multi_contour_polygons_[odom_ptr->layer_id]) {
                if (!poly_ptr->is_pillar && DPUtil::PointInsideAPoly(poly_ptr->vertices, p)) {
                    is_sample_free = false;
                    break;
                }
            }
            if (is_sample_free) {
                global_free_p = p;
                break;
            }
        }
    }
}

ConnectPair ContourGraph::ReprojectEdge(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2, const float& dist, const bool& is_inv) {
    ConnectPair edgeOut;
    cv::Point2f node1_cv, node2_cv;
    cv::Point2f dir1, dir2;
    const float norm = (node_ptr1->position - node_ptr2->position).norm();
    float ref_dist1, ref_dist2;
    ref_dist1 = ref_dist2 = dist;
    // node 1
    if (node_ptr1->is_contour_match && node_ptr1->ctnode->free_direct == node_ptr1->free_direct) {
        const auto ctnode1 = node_ptr1->ctnode;
        node1_cv = cv::Point2f(ctnode1->position.x, ctnode1->position.y);
        if (!is_inv) dir1 = NodeProjectDir(ctnode1);
        else dir1 = -NodeProjectDir(ctnode1);
        ref_dist1 = DPUtil::kLeafSize;
    } else {
        // if (node_ptr1->is_odom) {
        //     node1_cv = cv::Point2f(DPUtil::free_odom_p.x, DPUtil::free_odom_p.y);
        // } else {
        //     node1_cv = cv::Point2f(node_ptr1->position.x, node_ptr1->position.y);
        // }  
        node1_cv = cv::Point2f(node_ptr1->position.x, node_ptr1->position.y); 
        // dir1 = NodeProjectDir(node_ptr1);
        if (!is_inv) dir1 = NodeProjectDir(node_ptr1);
        else {
            dir1 = -NodeProjectDir(node_ptr1);
        }
        ref_dist1 = std::min(norm*(float)0.4, ref_dist1);
    }
    edgeOut.start_p = node1_cv + ref_dist1 * dir1;
    // node 2
    if (node_ptr2->is_contour_match && node_ptr2->ctnode->free_direct == node_ptr2->free_direct) {
        const auto ctnode2 = node_ptr2->ctnode;
        node2_cv = cv::Point2f(ctnode2->position.x, ctnode2->position.y);
        dir2 = NodeProjectDir(ctnode2);
        ref_dist2 = DPUtil::kLeafSize;
    } else {
        // if (node_ptr2->is_odom) {
        //     node2_cv = cv::Point2f(DPUtil::free_odom_p.x, DPUtil::free_odom_p.y);
        // } else {
        //     node2_cv = cv::Point2f(node_ptr2->position.x, node_ptr2->position.y);
        // }
        node2_cv = cv::Point2f(node_ptr2->position.x, node_ptr2->position.y);
        dir2 = NodeProjectDir(node_ptr2);
        ref_dist2 = std::min(norm*(float)0.4, dist);
    }
    edgeOut.end_p = node2_cv + ref_dist2 * dir2;
    return edgeOut;
}

void ContourGraph::BuildKDTreeOnContourGraph(const CTNodeStack& contour_graph, const int& layer_id) {
    cv::Mat contour_mat(contour_graph.size(), 2, CV_32FC1);
    for (size_t i = 0; i < contour_graph.size(); ++i) {
        contour_mat.at<float>(i, 0) = contour_graph[i]->position.x;
        contour_mat.at<float>(i, 1) = contour_graph[i]->position.y;
    }
    if (contour_mat.empty()) return;

    multi_contour_graph_kdtree_[layer_id].reset(new cv::flann::Index(contour_mat, *indexParams));
    is_kdtree_built_= true;
    // TestContourGraph(layer_id);
}

void ContourGraph::SearchKNN(const CTNodePtr& node, std::vector<int>& indices, const int& layer_id) {
    cv::Mat node_mat(1, 2, CV_32FC1);
    node_mat.at<float>(0, 0) = node->position.x;
    node_mat.at<float>(0, 1) = node->position.y;
    cv::Mat indices_mat, dists;
    multi_contour_graph_kdtree_[layer_id]->knnSearch(node_mat, indices_mat, dists, ctgraph_params_.knn_search_num_);
    // Transfer the results back
    for (int i = 0; i < indices_mat.rows; i++) {
        for (int j = 0; j < indices_mat.cols; j++) {
            int index = indices_mat.at<int>(i, j);
            float distance = std::sqrt(dists.at<float>(i, j));
            if (distance < ctgraph_params_.knn_search_radius_) {
                // cout<<"distance: "<<distance<<endl;
                indices.push_back(index);
            }
            // std::cout<<"========================"<<std::endl;
            // cout<<"contour_graph size: "<<contour_graph.size()<<endl;
            // std::cout << "Index: " << index 
            //           << ", Distance: " << sqrt(distance)
            //           << ", GraphPoint: (" << multi_contour_graph_[layer_id][index]->position.x << ", " << multi_contour_graph_[layer_id][index]->position.y << ", " << multi_contour_graph_[layer_id][index]->position.z
            //           << ", node: (" << node->position.x << ", " << node->position.y << ")"
            //           << std::endl;
            // std::cout<<"========================"<<std::endl;
        }
    }    
}

void ContourGraph::ConnectVerticalEdges(const int& layer_id) {
    for (const auto& node_ptr : multi_contour_graph_[layer_id]) {
        std::vector<int> indices;
        SearchKNN(node_ptr, indices, layer_id+1);
        // ROS_WARN("indices size: %d", indices.empty());

        for (const auto& index : indices) {
            if (index == -1 || index >= multi_contour_graph_[layer_id+1].size()) break;
            const auto& neighbor_ptr = multi_contour_graph_[layer_id+1][index];
            if (neighbor_ptr == NULL) continue;
            // ROS_WARN("neighbor_ptr is not NULL");
            if (node_ptr->free_direct == NodeFreeDirect::PILLAR || neighbor_ptr->free_direct == NodeFreeDirect::PILLAR) continue;
            if (node_ptr->free_direct == neighbor_ptr->free_direct && (node_ptr->position - neighbor_ptr->position).norm() < 1.5) {
                Point3D topo_dir = node_ptr->surf_dirs.first + node_ptr->surf_dirs.second;
                topo_dir.z = 0.0;
                if (topo_dir.norm() < DPUtil::kEpsilon) continue;
                topo_dir = neighbor_ptr->surf_dirs.first + neighbor_ptr->surf_dirs.second;
                topo_dir.z = 0.0;
                if (topo_dir.norm() < DPUtil::kEpsilon) continue;

                const Point3D node_dir = DPUtil::SurfTopoDirect(node_ptr->surf_dirs);
                const Point3D neighbor_dir = DPUtil::SurfTopoDirect(neighbor_ptr->surf_dirs);

                const float cos_theta = node_dir*neighbor_dir;

                if (cos_theta > 0.5) {
                    node_ptr->up = neighbor_ptr;
                    neighbor_ptr->down = node_ptr;
                    AddConnect(node_ptr, neighbor_ptr);
                    break;
                }
            }
        }
    }
}

void ContourGraph::SetWallCornerNodes(const CTNodePtr& node_ptr1, const CTNodePtr& node_ptr2, const int& layer_idx) {
    // if (node_ptr1->is_wall_corner || node_ptr2->is_wall_corner) {ROS_ERROR("SetWallCornerNodes: node_ptr1 or node_ptr2 is already wall corner node"); return;}

    node_ptr1->is_wall_corner = true;
    // node_ptr2->is_wall_corner = true;

    bool is_insert_finished = false;
    CTNodePtr start = node_ptr1;
    CTNodePtr end = node_ptr2;
    while (!is_insert_finished)
    {
        if (InsertWallNodes(start, end, layer_idx)) {
            is_insert_finished = true;
        }


        // if (InsertWallNodes(start, end, layer_idx)) {
        //     is_insert_finished = true;
        // } else {
        //     // if (InsertWallNodes(end, start, layer_idx)) {
        //     //     is_insert_finished = true;
        //     // }
        // }
    }
}

bool ContourGraph::InsertWallNodes(CTNodePtr& start, CTNodePtr& end, const int& layer_idx) {
    // insert nodes from node2 side
    Point3D dir = (end->position - start->position).normalize();
    Point3D insert_node = start->position + dir * DPUtil::kSensorRange*ctgraph_params_.wall_insert_factor;
    CTNodePtr insert_node_ptr = NULL;
    CreateCTNode(insert_node, start->layer_id, insert_node_ptr, start->poly_ptr, false);
    insert_node_ptr->free_direct = NodeFreeDirect::INSERT;

    if (start->front == end) {
        start->front = insert_node_ptr;
        insert_node_ptr->front = end;
        end->back = insert_node_ptr;
        insert_node_ptr->back = start;
        insert_node_ptr->is_wall_insert = true;
        insert_node_ptr->poly_ptr->is_connect = true;
        // insert_node_ptr->is_global_match = true;
        // start->poly_ptr->vertices.push_back(insert_node_ptr->position);

        this->AddCTNodeToGraph(insert_node_ptr, layer_idx);
        start = insert_node_ptr;

        if ((insert_node_ptr->position - end->position).norm() > DPUtil::kSensorRange*ctgraph_params_.wall_insert_factor) {
            // cout<<"Current dis: "<<(start->position - end->position).norm()<<endl;
            return false;
        }
    } 
    else if (start->back == end) {
        end->front = insert_node_ptr;
        insert_node_ptr->front = start;
        start->back = insert_node_ptr;
        insert_node_ptr->back = end;
        insert_node_ptr->is_wall_insert = true;
        insert_node_ptr->poly_ptr->is_connect = true;
        // insert_node_ptr->is_global_match = true;
        // start->poly_ptr->vertices.push_back(insert_node_ptr->position);

        this->AddCTNodeToGraph(insert_node_ptr, layer_idx);
        start = insert_node_ptr;

        if ((insert_node_ptr->position - end->position).norm() > DPUtil::kSensorRange*ctgraph_params_.wall_insert_factor) {
            // cout<<"Current dis: "<<(start->position - end->position).norm()<<endl;
            return false;
        }
    } 
    // else {
    //     ROS_ERROR("InsertWallNodes: start and end node are not connected");
    //     return false;
    // }
    // cout<<"EXIT dis: "<<(start->position - end->position).norm()<<endl;
    return true;

}

bool ContourGraph::IsTopLayerNodesConnectFreePolygon(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2, const bool& layer_limited) {
    // if (node_ptr1->is_navpoint || node_ptr2->is_navpoint) {
    //     if (node_ptr1->layer_id == node_ptr2->layer_id && (node_ptr1->position - node_ptr2->position).norm() < DPUtil::kNavClearDist) { // connect to internav node
    //         return true;
    //     }
    // }
    bool is_layer_limit = layer_limited;
    if (node_ptr1->is_odom || node_ptr2->is_odom) is_layer_limit = false;
    ConnectPair cedge;
    int start_layer, end_layer;
    if (node_ptr1->layer_id == node_ptr2->layer_id) {
        cedge = ContourGraph::ReprojectEdge(node_ptr1, node_ptr2, DPUtil::kNavClearDist, false);
        start_layer = node_ptr1->layer_id+1, end_layer = node_ptr2->layer_id+1;
    } else if (node_ptr1->layer_id < node_ptr2->layer_id) {
        cedge = ContourGraph::ReprojectEdge(node_ptr1, node_ptr2, DPUtil::kNavClearDist, false);
        start_layer = node_ptr1->layer_id+1, end_layer = node_ptr2->layer_id;
    } else {
        cedge = ContourGraph::ReprojectEdge(node_ptr2, node_ptr1, DPUtil::kNavClearDist, false);
        start_layer = node_ptr2->layer_id+1, end_layer = node_ptr1->layer_id;
    }
    bool is_global_check = false;
    // if (!is_local_only && (!node_ptr1->is_contour_match || !node_ptr2->is_contour_match) && 
    //     ContourGraph::IsNeedGlobalCheck(node_ptr1->position, node_ptr2->position, DPUtil::odom_pos)) 
    // {
    //     is_global_check = true;
    // }
    return ContourGraph::IsPointsConnectFreePolygon(cedge, start_layer, end_layer, is_global_check, layer_limited);
}


/************************** UNUSE CODE ****************************/

// bool ContourGraph::ReprojectPointOutsidePolygons(Point3D& point, const float& free_radius) {
    // PolygonPtr inside_poly_ptr = NULL;
    // bool is_inside_poly = false;
    // for (const auto& poly_ptr : ContourGraph::contour_polygons_) {
    //     if (poly_ptr->is_pillar) continue;
    //     if (DPUtil::PointInsideAPoly(poly_ptr->vertices, point) && !DPUtil::PointInsideAPoly(poly_ptr->vertices, DPUtil::free_odom_p)) {
    //         inside_poly_ptr = poly_ptr;
    //         is_inside_poly = true;
    //         break;
    //     }
    // }
    // if (is_inside_poly) {
    //     float near_dist = DPUtil::kINF;
    //     Point3D reproject_p = point;
    //     Point3D free_dir(0,0,-1);
    //     const int N = inside_poly_ptr->vertices.size();
    //     for (int idx=0; idx<N; idx++) {
    //         const Point3D vertex = inside_poly_ptr->vertices[idx];
    //         const float temp_dist = (vertex - point).norm();
    //         if (temp_dist < near_dist) {
    //             const Point3D dir1 = (inside_poly_ptr->vertices[DPUtil::Mod(idx-1, N)] - vertex).normalize();
    //             const Point3D dir2 = (inside_poly_ptr->vertices[DPUtil::Mod(idx+1, N)] - vertex).normalize();
    //             const Point3D dir = (dir1 + dir2).normalize();
    //             if (DPUtil::PointInsideAPoly(inside_poly_ptr->vertices, vertex + dir * DPUtil::kLeafSize)) { // convex 
    //                 reproject_p = vertex;
    //                 near_dist = temp_dist;
    //                 free_dir = dir;
    //             }
    //         }
    //     }
    //     const float origin_z = point.z;
    //     point = reproject_p - free_dir * free_radius;
    //     point.z = origin_z;
    // }
    // return is_inside_poly;
// }

// bool ContourGraph::IsPoint3DConnectFreePolygon(const Point3D& p1, const Point3D& p2, const int& start_layer, const int& end_layer) {
//     const bool is_global_check = ContourGraph::IsNeedGlobalCheck(p1, p2, DPUtil::odom_pos);
//     const ConnectPair ori_cedge(p1, p2);
//     const ConnectPair cedge = ori_cedge;
//     if (end_layer - start_layer > DPUtil::kNeighborLayers) return false; // max acrossing layers
//     return ContourGraph::IsPointsConnectFreePolygon(cedge, ori_cedge, 
//                                                     start_layer, end_layer,
//                                                     NULL, NULL, 
//                                                     is_global_check);
// }

// test functions

// // test knn search
// void TestContourGraph(const int& layer_id) {
//     cv::Mat odom_mat(1, 2, CV_32FC1);
//     odom_mat.at<float>(0, 0) = odom_node_ptr_->position.x;
//     odom_mat.at<float>(0, 1) = odom_node_ptr_->position.y;
//     cv::Mat indices, dists;
//     multi_contour_graph_kdtree_[layer_id]->knnSearch(odom_mat, indices, dists, 1);
//     // Print out the results
//     for (int i = 0; i < indices.rows; i++)
//     {
//         for (int j = 0; j < indices.cols; j++)
//         {
//             int index = indices.at<int>(i, j);
//             float x = contour_mat.at<float>(index, 0);
//             float y = contour_mat.at<float>(index, 1);
//             float distance = dists.at<float>(i, j);
//             std::cout<<"========================"<<std::endl;
//             cout<<"contour_graph size: "<<contour_graph.size()<<endl;
//             std::cout << "Index: " << index 
//                       << ", Distance: " << sqrt(distance)
//                       << ", GraphPoint: (" << contour_graph[index]->position.x << ", " << contour_graph[index]->position.y << ", " << contour_graph[index]->position.z
//                       << ", odom: (" << odom_node_ptr_->position.x << ", " << odom_node_ptr_->position.y << ")"
//                       << std::endl;
//             std::cout<<"========================"<<std::endl;
//         }
//     }
// }
