/*
 * Dynamic Route Planner
 * Copyright (C) 2021 Fan Yang - All rights reserved
 * fanyang2@andrew.cmu.edu,   
 */



#include "airfar_planner/dynamic_graph.h"

/***************************************************************************************/

void DynamicGraph::Init(const ros::NodeHandle& nh, const DynamicGraphParams& params) {
    dg_params_ = params;
    near_nav_nodes_.clear();
    wide_near_nodes_.clear();
    CONNECT_ANGLE_COS = cos(dg_params_.kConnectAngleThred);
    NOISE_ANGLE_COS = cos(DPUtil::kAngleNoise);
    ALIGN_ANGLE_COS = cos(M_PI - DPUtil::kAcceptAlign / 1.5);
    VERTICAL_ANGLE_COS = sin(DPUtil::kVerticalAngleThred);
    TRAJ_DIST = dg_params_.sensor_range / dg_params_.traj_interval_ratio;
    id_tracker_ = 2;
    last_connect_pos_ = Point3D(0,0,0);
    /* Initialize Terrian Planner */
    tp_params_.world_frame = DPUtil::worldFrameId;
    tp_params_.local_range = TRAJ_DIST;
    tp_params_.voxel_size = DPUtil::kLeafSize;
    tp_params_.radius = DPUtil::kNearDist * 2.0;
    tp_params_.inflate_size = dg_params_.terrian_inflate;
    terrain_planner_.Init(nh, tp_params_);
}

bool DynamicGraph::UpdateOdom(const Point3D& robot_pos, const std::vector<int>& cur_layers, bool& is_robot_stop) {
    robot_pos_ = robot_pos;
    bool is_odom_node_updated = false;
    terrain_planner_.SetLocalTerrainObsCloud(DPUtil::local_terrain_obs_);
    cur_layer_idxs_ = cur_layers;
    this->ConvertWideLayerIdxs(cur_layer_idxs_, wide_layer_idxs_);
    const int odom_layer_id = cur_layer_idxs_[DPUtil::kNeighborLayers];
    if (odom_node_ptr_ == NULL) {
        this->CreateNavNodeFromPoint(robot_pos_, odom_layer_id, odom_node_ptr_, true);
        this->AddNodeToGraph(odom_node_ptr_);
        ROS_INFO("DG: Odom navigation node initilaized.");
        is_odom_node_updated = true;
    } else {
        const Point3D pos_diff = odom_node_ptr_->position - robot_pos_;
        if (pos_diff.norm() > dg_params_.move_thred) {
            this->UpdateNodePosition(odom_node_ptr_, robot_pos_);
            ROS_INFO("DG: Odom graph updated");
            is_odom_node_updated = true;
        } else {
            ROS_WARN_COND(!is_robot_stop, "DG: Wait for robot to move...");
            is_odom_node_updated = false;
            is_robot_stop = true;
        }
    }
    if (is_odom_node_updated) {
        DPUtil::odom_pos = odom_node_ptr_->position;
        odom_node_ptr_->layer_id = odom_layer_id;
        is_robot_stop = false;
        this->UpdateGlobalNearNodes();
        terrain_planner_.VisualPaths();
        return true;
    }
    return false;
}

bool DynamicGraph::IsInterNavpointNecessary() {
    if (cur_internav_ptr_ == NULL || is_bridge_internav_) { // create nearest nav point
        if (cur_internav_ptr_ == NULL) {
            last_connect_pos_ = DPUtil::free_odom_p;
        }
        return true;
    }
    const auto it = odom_node_ptr_->edge_votes.find(cur_internav_ptr_->id);
    if (it == odom_node_ptr_->edge_votes.end() || !this->IsInternavInRange(cur_internav_ptr_)) {
        return true;
    } else if (it != odom_node_ptr_->edge_votes.end() && it->second.back() == 1) {
        last_connect_pos_ = DPUtil::free_odom_p;
    }
    return false;
}

bool DynamicGraph::IsOldNodesAround(const CTNodePtr& ctnode, const float& radius) {
    // ROS_WARN("DG: IsOldNodesAround");
    Point3D ref_topo_dir(0,0,0);
    if (ctnode->free_direct != NodeFreeDirect::PILLAR) ref_topo_dir = DPUtil::SurfTopoDirect(ctnode->surf_dirs);
    for (const auto& node_ptr : near_nav_nodes_) {
        if (node_ptr->layer_id != ctnode->layer_id || DPUtil::IsFreeNavNode(node_ptr) || !node_ptr->is_finalized) continue;
        if (node_ptr->is_contour_match && node_ptr->ctnode->poly_ptr != ctnode->poly_ptr) continue;
        const float dist = (ctnode->position - node_ptr->position).norm();
        if (dist < radius && ctnode->free_direct == node_ptr->free_direct) {
            if (ctnode->free_direct == NodeFreeDirect::PILLAR) {
                return true;
            } else if (ref_topo_dir.norm() > DPUtil::kEpsilon) {
                const Point3D topo_dir = DPUtil::SurfTopoDirect(node_ptr->surf_dirs);
                if (topo_dir.norm() > DPUtil::kEpsilon && topo_dir*ref_topo_dir > NOISE_ANGLE_COS) {
                    return true;
                }
            }       
        } 
    }
    return false;
}

bool DynamicGraph::IsOldWallNodesAround(const CTNodePtr& ctnode, const float& radius) {
    // ROS_WARN("DG: IsOldWallNodesAround");
    Point3D ref_topo_dir(0,0,0);
    if (ctnode->free_direct != NodeFreeDirect::PILLAR) ref_topo_dir = DPUtil::SurfTopoDirect(ctnode->surf_dirs);
    for (const auto& node_ptr : wide_near_nodes_) {
        if (!node_ptr->is_wall_insert || node_ptr->layer_id != ctnode->layer_id || DPUtil::IsFreeNavNode(node_ptr) || !node_ptr->is_finalized) continue;
        if (node_ptr->is_contour_match && node_ptr->ctnode->poly_ptr != ctnode->poly_ptr) continue;
        const float dist = (ctnode->position - node_ptr->position).norm();
        if (dist < radius && ctnode->free_direct == node_ptr->free_direct) {
            if (ctnode->free_direct == NodeFreeDirect::PILLAR) {
                return true;
            } else if (ref_topo_dir.norm() > DPUtil::kEpsilon) {
                const Point3D topo_dir = DPUtil::SurfTopoDirect(node_ptr->surf_dirs);
                if (topo_dir.norm() > DPUtil::kEpsilon && std::abs(topo_dir*ref_topo_dir) > NOISE_ANGLE_COS) {
                    return true;
                }
            }       
        } 
    }
    return false;
}

bool DynamicGraph::ExtractGraphNodes(const CTNodeStack& new_ctnodes) {
    if (new_ctnodes.empty() && !DPUtil::IsNavpoint) return false;
    NavNodePtr new_node_ptr = NULL;
    new_nodes_.clear();
    // check wheter or not need inter navigation points
    if (DPUtil::IsNavpoint && this->IsInterNavpointNecessary()) {
        ROS_WARN("DG: Intermediate navigation point is created.");
        this->CreateNavNodeFromPoint(last_connect_pos_, odom_node_ptr_->layer_id, new_node_ptr, false, true);
        new_nodes_.push_back(new_node_ptr);
        last_connect_pos_ = DPUtil::free_odom_p;
        if (is_bridge_internav_) is_bridge_internav_ = false;
    }
    for (const auto& ctnode_ptr : new_ctnodes) {
        if ((robot_pos_ - ctnode_ptr->position).norm() < dg_params_.sensor_range) {
            if ((DPUtil::IsPointNearNewPoints(ctnode_ptr->position, true) || ctnode_ptr->is_wall_insert) && !this->IsOldNodesAround(ctnode_ptr, DPUtil::kNavClearDist)) {
                if (ctnode_ptr->is_wall_insert && this->IsOldWallNodesAround(ctnode_ptr, DPUtil::kSensorRange*dg_params_.wall_insert_factor)) continue;
                this->CreateNewNavNodeFromContour(ctnode_ptr, new_node_ptr);
                new_nodes_.push_back(new_node_ptr);
            }
        }
    }
    if (new_nodes_.empty()) return false;
    else return true;
}

void DynamicGraph::UpdateNavGraph(const NodePtrStack& new_nodes,
                                  NodePtrStack& clear_node) 
{
    // clear false positive node detection
    clear_node.clear();
    for (const auto& node_ptr : near_nav_nodes_) {
        if (DPUtil::IsFreeNavNode(node_ptr)) continue;
        if (!this->ReEvaluateCorner(node_ptr)) {
            if (this->SetNodeToClear(node_ptr)) {
                clear_node.push_back(node_ptr);
            }
        } else {
            this->ReduceDumperCounter(node_ptr);
        }
    }
    // re-evaluate trajectory edge using terrain planner
    if (DPUtil::IsTrajectory && cur_internav_ptr_ != NULL) {
        NodePtrStack internav_check_nodes = surround_internav_nodes_;
        if (!DPUtil::IsTypeInStack(cur_internav_ptr_, internav_check_nodes)) {
            internav_check_nodes.push_back(cur_internav_ptr_);
        }
        for (const auto& sur_internav_ptr : internav_check_nodes) {
            const NodePtrStack copy_traj_connects = sur_internav_ptr->trajectory_connects;
            for (const auto& tnode_ptr : copy_traj_connects) {
            if (this->ReEvaluateConnectUsingTerrian(sur_internav_ptr, tnode_ptr)) {
                    this->RecordValidTrajEdge(sur_internav_ptr, tnode_ptr);
                } else {
                    this->RemoveInValidTrajEdge(sur_internav_ptr, tnode_ptr);
                }
            }   
        }     
    }
    // check-add connections to odom node with wider near nodes
    NodePtrStack codom_check_list = wide_near_nodes_;
    codom_check_list.insert(codom_check_list.end(), new_nodes.begin(), new_nodes.end());
    for (const auto& conode_ptr : codom_check_list) {
        if (conode_ptr->is_odom || conode_ptr->is_merged || conode_ptr->is_wall_insert) continue;
        // if (conode_ptr->free_direct == NodeFreeDirect::INSERT) ROS_ERROR("DG: Free direct: %d", conode_ptr->free_direct);
        if (DynamicGraph::IsConnectInVerticalConstrain(odom_node_ptr_, conode_ptr)
        && ContourGraph::IsNavToOdomConnectFreePolygon(conode_ptr, odom_node_ptr_)
        && this->IsValidConnect(odom_node_ptr_, conode_ptr, false, false, true)
        ) {
            this->AddEdge(odom_node_ptr_, conode_ptr);
            if (conode_ptr->is_contour_match) {
                conode_ptr->ctnode->poly_ptr->is_connect = true;
                conode_ptr->ctnode->poly_ptr->is_visiable = true;
            }
        } else {
            this->EraseEdge(conode_ptr, odom_node_ptr_);
        }
    }

    // DPUtil::Timer.start_time("reconnect between near nodes");

    // reconnect between near nodes
    for (std::size_t i=0; i<near_nav_nodes_.size(); i++) {
        if (near_nav_nodes_[i]->is_merged || near_nav_nodes_[i]->is_odom) continue;

        // re-evaluate nodes which are not in near
        const NodePtrStack copy_connect_nodes = near_nav_nodes_[i]->connect_nodes;
        for (const auto& cnode : copy_connect_nodes) {
            if (cnode->is_odom || cnode->is_near_nodes || DPUtil::IsTypeInStack(cnode, near_nav_nodes_[i]->contour_connects)) continue;
            if (this->IsValidConnect(near_nav_nodes_[i], cnode, true, false, false)) {
                this->AddEdge(near_nav_nodes_[i], cnode);
            } else {
                this->EraseEdge(near_nav_nodes_[i], cnode);
            } 
        }
        for (std::size_t j=0; j<near_nav_nodes_.size(); j++) {
            if (i == j || j > i || near_nav_nodes_[j]->is_merged || near_nav_nodes_[j]->is_odom) continue;
            bool is_merge = false;
            bool is_matched = false;
            if (!DPUtil::IsTypeInStack(near_nav_nodes_[i], near_nav_nodes_[j]->contour_connects) || !DPUtil::IsTypeInStack(near_nav_nodes_[j], near_nav_nodes_[i]->contour_connects)) {
                this->ConnectVerticalNodes(near_nav_nodes_[i], near_nav_nodes_[j]);
            }
            // ROS_WARN("2222222: %d", near_nav_nodes_.size());
            // bool ret = this->IsValidConnect(near_nav_nodes_[i], near_nav_nodes_[j], true, true, is_merge, is_matched, false);
            // ROS_WARN("3333333: %d", near_nav_nodes_.size());
            if (this->IsValidConnect(near_nav_nodes_[i], near_nav_nodes_[j], true, true, is_merge, is_matched, false)) {
                    if (is_matched || near_nav_nodes_[i]->is_navpoint || near_nav_nodes_[j]->is_navpoint) {
                        this->AddEdge(near_nav_nodes_[i], near_nav_nodes_[j]);
                    }
                } else if (is_merge) {
                    this->MergeNodeInGraph(near_nav_nodes_[i], near_nav_nodes_[j]);
                    if (near_nav_nodes_[i]->is_merged) {
                        clear_node.push_back(near_nav_nodes_[i]);
                        break;
                    } else {
                        clear_node.push_back(near_nav_nodes_[j]);
                    }
                } else {
                    this->EraseEdge(near_nav_nodes_[i], near_nav_nodes_[j]);
                }
        }
    }
    // cout<<"Clear node size: "<<clear_node.size()<<endl;
    // ROS_WARN("DG: Near nodes size: %d", near_nav_nodes_.size());

    // reconnect between vertical nodes
    // DPUtil::Timer.start_time("reconnect between Vertical nodes");
    for (std::size_t i=0; i<near_nav_nodes_.size(); i++) {
        const NodePtrStack copy_connect_nodes = near_nav_nodes_[i]->connect_nodes;
        if (near_nav_nodes_[i]->is_merged || near_nav_nodes_[i]->is_odom || copy_connect_nodes.empty()) continue;
        for (const auto& cnode : copy_connect_nodes) {
            if (cnode == NULL) {
                continue;
            }
            this->AddVerticalEdges(near_nav_nodes_[i], cnode);
        }

        // Identify top layer nodes
        if (near_nav_nodes_[i]->up_node == NULL && near_nav_nodes_[i]->down_node != NULL && (near_nav_nodes_[i]->position.z - odom_node_ptr_->position.z) < DPUtil::kTolerZ) {
            near_nav_nodes_[i]->is_top_layer = true;
            for (auto cnode : near_nav_nodes_[i]->contour_connects) {
                if (cnode->is_wall_insert) {
                    // cout<<"Is wall insert node connected: "<< cnode->connect_nodes.size()<<endl;
                    cnode->is_top_layer = true;
                }
            }
        } else if (near_nav_nodes_[i]->is_top_layer == true) {
            near_nav_nodes_[i]->is_top_layer = false;
            // for (auto cnode : near_nav_nodes_[i]->contour_connects) {
            //     if (cnode->is_wall_insert) {
            //         // cnode->is_top_layer = false;
            //         for (auto cnode2 : cnode->connect_nodes) {
            //             EraseEdge(cnode, cnode2);
            //         }
            //     }
            // }
        }

        // remove connections of non-top layer inserted wall nodes
        // if (near_nav_nodes_[i]->is_wall_insert && near_nav_nodes_[i]->is_top_layer == false) {
        if (near_nav_nodes_[i]->is_wall_insert) {
            for (auto cnode : near_nav_nodes_[i]->connect_nodes) {
                DPUtil::EraseNodeFromStack(near_nav_nodes_[i], cnode->connect_nodes);
            }
            near_nav_nodes_[i]->connect_nodes.clear();
        }

        // Identify bottom layer nodes
        if (near_nav_nodes_[i]->down_node == NULL && near_nav_nodes_[i]->up_node != NULL)
        //  && (near_nav_nodes_[i]->position.z - odom_node_ptr_->position.z) > DPUtil::kTolerZ) 
        {
            near_nav_nodes_[i]->is_bottom_layer = true;
            for (auto cnode : near_nav_nodes_[i]->connect_nodes) {
                DPUtil::EraseNodeFromStack(near_nav_nodes_[i], cnode->connect_nodes);
            }
            near_nav_nodes_[i]->connect_nodes.clear();
        } else if (near_nav_nodes_[i]->is_bottom_layer == true) {
            near_nav_nodes_[i]->is_bottom_layer = false;
        }

        // remove connections between vertical contours
        if (near_nav_nodes_[i]->up_node != NULL) this->EraseEdge(near_nav_nodes_[i], near_nav_nodes_[i]->up_node);
        if (near_nav_nodes_[i]->down_node != NULL) this->EraseEdge(near_nav_nodes_[i], near_nav_nodes_[i]->down_node);
    }
    // DPUtil::Timer.end_time("reconnect between Vertical nodes");
    // DPUtil::Timer.end_time("reconnect between near nodes");

    // DPUtil::Timer.start_time("Adding edges between existing nodes with new extracted nodes");
    // Adding edges between existing nodes with new extracted nodes
    for (const auto& new_node_ptr : new_nodes) {
        if (!this->IsConnectedNewNode(new_node_ptr) || new_node_ptr->is_wall_insert) continue;
        for (const auto& neighbor_node_ptr : near_nav_nodes_) {
            if (neighbor_node_ptr->is_odom || neighbor_node_ptr->is_merged || neighbor_node_ptr->is_wall_insert) continue;
            if (this->IsValidConnect(neighbor_node_ptr, new_node_ptr, true)) {
                this->AddEdge(neighbor_node_ptr, new_node_ptr);
            }
        }
        this->AddNodeToGraph(new_node_ptr);
        if (new_node_ptr->is_navpoint) {
            this->UpdateCurInterNavNode(new_node_ptr);
        }
    }
    // DPUtil::Timer.end_time("Adding edges between existing nodes with new extracted nodes");
    this->ClearMergedNodesInGraph();
    // Analysisig frontier nodes
    for (const auto& cnode_ptr : near_nav_nodes_) {
        if (this->IsNodeFullyCovered(cnode_ptr)) {
            cnode_ptr->is_frontier = false;
        } else if (!cnode_ptr->is_navpoint) {
            cnode_ptr->is_frontier = true;
        }
    }
    ROS_INFO("DG: Graph Updated, current number of navigation nodes: %ld", globalGraphNodes_.size());
}

bool DynamicGraph::IsValidConnect(const NavNodePtr& node_ptr1, 
                                  const NavNodePtr& node_ptr2,
                                  const bool& is_local_only,
                                  const bool& is_check_contour,
                                  bool& _is_merge,
                                  bool& _is_matched,
                                  const bool& is_layer_limited)
{
    // TODO: Can be return true
    if ((node_ptr1->position - node_ptr2->position).norm() < DPUtil::kEpsilon) {
        _is_merge = true;
        return false;
    }
    
    if (node_ptr1->is_contour_match && node_ptr2->is_contour_match) {
        _is_matched = true;
    } else {
        _is_matched = false;
    }
    if (!DPUtil::IsFreeNavNode(node_ptr1) && !DPUtil::IsFreeNavNode(node_ptr2) && node_ptr1->layer_id == node_ptr2->layer_id) {
        const float dist = (node_ptr1->position - node_ptr2->position).norm();
        if (dist < DPUtil::kNavClearDist && node_ptr1->free_direct == node_ptr2->free_direct) {
            bool is_same_dir = true;
            if (node_ptr1->free_direct != NodeFreeDirect::PILLAR) {
                const Point3D topo_dir1 = DPUtil::SurfTopoDirect(node_ptr1->surf_dirs);
                const Point3D topo_dir2 = DPUtil::SurfTopoDirect(node_ptr2->surf_dirs);
                if (topo_dir1 * topo_dir2 < NOISE_ANGLE_COS || node_ptr1->is_wall_insert || node_ptr2->is_wall_insert) is_same_dir = false;
            }
            if (is_same_dir) {
                _is_merge = true;
                return false;
            }
        }
    }   

    if (!DynamicGraph::IsConnectInVerticalConstrain(node_ptr1, node_ptr2)) {
        return false;
    }

    _is_merge = false;
    /* check contour connection from node1 to node2 */
    bool is_connect = false;
    if (!is_connect && is_check_contour && node_ptr1->layer_id == node_ptr2->layer_id) {
        // if (node_ptr1->is_wall_insert || node_ptr2->is_wall_insert) {
        //     cout<<"DG: Contour Record: " << ContourGraph::IsNavNodesConnectFromContour(node_ptr1, node_ptr2) << endl;
        // }

        if (ContourGraph::IsNavNodesConnectFromContour(node_ptr1, node_ptr2) && 
        (this->IsInContourDirConstraint(node_ptr1, node_ptr2) || node_ptr1->is_wall_insert || node_ptr2->is_wall_insert)) {
            this->RecordContourEdge(node_ptr1, node_ptr2);
        }
        else if (node_ptr1->is_contour_match && node_ptr2->is_contour_match) {
            this->DeleteContourEdge(node_ptr1, node_ptr2);
        } 

    }

    if (DPUtil::IsTypeInStack(node_ptr1, node_ptr2->contour_connects)) is_connect = true;
    /* check if the trajectory connect is recorded but not valid */
    bool is_invalid_traj = false;
    if (DPUtil::IsTrajectory && node_ptr1->is_navpoint && node_ptr2->is_navpoint) {
        const auto it = node_ptr1->trajectory_votes.find(node_ptr2->id);
        if (it != node_ptr1->trajectory_votes.end() && !DPUtil::IsVoteTrue(it->second)) {
            is_invalid_traj = true;
        } 
    }

    /* check polygon connections */
    if (!is_connect && !is_invalid_traj) {
        bool is_valid_edge = false;
        const int vote_queue_size = (node_ptr1->is_odom || node_ptr2->is_odom) ? std::ceil(dg_params_.votes_size/3.0) : dg_params_.votes_size;
        if (((node_ptr1->free_direct != NodeFreeDirect::CONCAVE || node_ptr2->is_odom) && 
             (node_ptr2->free_direct != NodeFreeDirect::CONCAVE || node_ptr1->is_odom)) || 
            (node_ptr1->is_top_layer || node_ptr2->is_top_layer))
        {
            if (node_ptr1->is_wall_insert || node_ptr2->is_wall_insert || (this->IsInDirectConstraint(node_ptr1, node_ptr2) && 
            ContourGraph::IsNavNodesConnectFreePolygon(node_ptr1, node_ptr2, is_local_only, is_layer_limited))) {
                is_valid_edge = true;
                this->RecordPolygonEdge(node_ptr1, node_ptr2, vote_queue_size);
            }
        }
        if (!is_valid_edge) this->DeletePolygonEdge(node_ptr1, node_ptr2, vote_queue_size);
        if (this->IsPolygonEdgeVoteTrue(node_ptr1, node_ptr2)) {
            if (!this->IsSimilarConnectInDiection(node_ptr1, node_ptr2) 
            || node_ptr1->is_wall_insert || node_ptr2->is_wall_insert) is_connect = true;
        } else if (node_ptr1->is_odom || node_ptr2->is_odom) {
            node_ptr1->edge_votes.erase(node_ptr2->id);
            node_ptr2->edge_votes.erase(node_ptr1->id);
            // clear potential connections
            DPUtil::EraseNodeFromStack(node_ptr2, node_ptr1->potential_edges);
            DPUtil::EraseNodeFromStack(node_ptr1, node_ptr2->potential_edges);
        }
    }

    /* check if exsiting trajectory connection exist */
    if (DPUtil::IsTrajectory && !is_connect) {
        if (DPUtil::IsTypeInStack(node_ptr1, node_ptr2->trajectory_connects)) is_connect = true;
        if ((node_ptr1->is_odom || node_ptr2->is_odom) && cur_internav_ptr_ != NULL) {
            if ((odom_node_ptr_->position - cur_internav_ptr_->position).norm() < DPUtil::kNearDist) {
                if (node_ptr1->is_odom && DPUtil::IsTypeInStack(node_ptr2, cur_internav_ptr_->trajectory_connects)) {
                    is_connect = true;
                } else if (node_ptr2->is_odom && DPUtil::IsTypeInStack(node_ptr1, cur_internav_ptr_->trajectory_connects)) {
                    is_connect = true;
                }
            }   
        }
    }

    if (node_ptr1->is_top_layer && node_ptr2->is_top_layer) {
        if (ContourGraph::IsTopLayerNodesConnectFreePolygon(node_ptr1, node_ptr2, true)) {
            is_connect = true;
        } else {
            is_connect = false;
        }
    }
    // if (node_ptr1->is_wall_insert && node_ptr2->is_wall_insert && _is_merge) {
    //     cout<<"DG: BOTH wall: " << is_connect << _is_merge << endl;
    //     // is_connect = true;
    // } else if ((node_ptr1->is_wall_insert || node_ptr2->is_wall_insert) && _is_merge) {
    //     cout<<"DG: ONE of wall: " << is_connect << _is_merge <<endl;
    //     // is_connect = true;
    // }

    return is_connect;
    // return true;

}

bool DynamicGraph::IsValidConnect(const NavNodePtr& node_ptr1, 
                                  const NavNodePtr& node_ptr2,
                                  const bool& is_local_only,
                                  const bool& is_check_contour,
                                  const bool& is_layer_limited) 
{
    bool UNUSE_is_merge, UNUSE_is_matched;
    return this->IsValidConnect(node_ptr1, node_ptr2, is_local_only, is_check_contour, UNUSE_is_merge, UNUSE_is_matched, is_layer_limited);
}

void DynamicGraph::MergeNodeInGraph(const NavNodePtr& node_ptr1, 
                                    const NavNodePtr& node_ptr2) 
{
//  DEBUG
    // // if (node_ptr1->is_wall_insert || node_ptr2->is_wall_insert) {
    //     ROS_ERROR_STREAM("Is_finalized: "<<node_ptr1->is_finalized<<" "<<node_ptr2->is_finalized);
    //     ROS_ERROR("====================================");
    // // }
    if (DPUtil::IsFreeNavNode(node_ptr1) || DPUtil::IsFreeNavNode(node_ptr2)) return;
    const bool is_node1_outreach = DPUtil::IsOutReachNode(node_ptr1) ? true : false;
    const bool is_node2_outreach = DPUtil::IsOutReachNode(node_ptr2) ? true : false;
    if (is_node1_outreach || is_node2_outreach) {
        if (is_node1_outreach && is_node2_outreach) return;
        if (is_node1_outreach) node_ptr2->is_merged = true;
        else node_ptr1->is_merged = true;
    }

    if (!node_ptr1->is_merged && !node_ptr2->is_merged) {
        if (node_ptr1->is_contour_match && node_ptr2->is_contour_match) {
            return;
        } else if (node_ptr1->is_contour_match && node_ptr1->is_finalized) {
            node_ptr2->is_merged = true;
        } else if (node_ptr2->is_contour_match && node_ptr2->is_finalized) {
            node_ptr1->is_merged = true;
        } else {
            // compare contour connections
            const int conntour_c1 = node_ptr1->contour_connects.size();
            const int conntour_c2 = node_ptr2->contour_connects.size();
            if (conntour_c1 < conntour_c2) node_ptr1->is_merged = true;
            else if (conntour_c1 > conntour_c2) node_ptr2->is_merged = true;
            else {
                // compare finalization
                const int finalize_c1 = node_ptr1->pos_filter_vec.size() + node_ptr1->surf_dirs_vec.size();
                const int finalize_c2 = node_ptr2->pos_filter_vec.size() + node_ptr2->surf_dirs_vec.size();
                if (finalize_c1 < finalize_c2) {
                    node_ptr1->is_merged = true;
                } else {
                    node_ptr2->is_merged = true;
                }
            }
        }
    }
    // assign frontier flag
    if (!node_ptr1->is_frontier && node_ptr1->is_merged) node_ptr2->is_frontier = false;
    if (!node_ptr2->is_frontier && node_ptr2->is_merged) node_ptr1->is_frontier = false;
}

bool DynamicGraph::IsNodeFullyCovered(const NavNodePtr& node_ptr) {
    if (DPUtil::IsFreeNavNode(node_ptr) || !node_ptr->is_frontier) return true;
    NodePtrStack check_odom_list = internav_near_nodes_;
    check_odom_list.push_back(odom_node_ptr_);
    for (const auto& near_optr : check_odom_list) {
        const float cur_dist = (node_ptr->position - near_optr->position).norm();
        if (cur_dist < DPUtil::kNearDist) return true;
        else if (cur_dist > dg_params_.sensor_range) return false;
        if (node_ptr->free_direct != NodeFreeDirect::PILLAR) {
            if (node_ptr->surf_dirs_vec.size() > 2 && DPUtil::IsTypeInStack(node_ptr, near_optr->connect_nodes)) {
                const Point3D diff_p = near_optr->position - node_ptr->position;
                if (diff_p.norm() < DPUtil::kNearDist || DPUtil::IsInCoverageDirPairs(diff_p, node_ptr)) {
                    return true;
                }
            }
        }
    }
    return false;
}

bool DynamicGraph::IsSimilarConnectInDiection(const NavNodePtr& node_ptr_from,
                                              const NavNodePtr& node_ptr_to)
{
    if (node_ptr_from->is_odom || node_ptr_to->is_odom || node_ptr_from->layer_id != node_ptr_to->layer_id) {
        return false;
    }
    if (DPUtil::IsTypeInStack(node_ptr_to, node_ptr_from->contour_connects)) { // release for contour connection
        return false;
    }
    if (node_ptr_from->is_wall_insert || node_ptr_to->is_wall_insert) {
        return false;
    }
    // check from to to node connection
    if (this->IsAShorterConnectInDir(node_ptr_from, node_ptr_to)) {
        return true;
    }
    if (this->IsAShorterConnectInDir(node_ptr_to, node_ptr_from)) {
        return true;
    }
    return false;
}

bool DynamicGraph::IsInDirectConstraint(const NavNodePtr& node_ptr1,
                                        const NavNodePtr& node_ptr2) 
{
    if (DPUtil::IsFreeNavNode(node_ptr1) || DPUtil::IsFreeNavNode(node_ptr2)) return true;
    // check maximum vertical angle
    if (!DynamicGraph::IsConnectInVerticalConstrain(node_ptr1, node_ptr2)) return false;
    if (node_ptr1->is_wall_insert || node_ptr2->is_wall_insert) return true;
    // check node1 -> node2
    bool is_node1_valid = true;
    if (node_ptr1->free_direct != NodeFreeDirect::PILLAR) {
        Point3D diff_1to2 = (node_ptr2->position - node_ptr1->position);
        if (!DPUtil::IsInSurfacePairs(diff_1to2, node_ptr1->surf_dirs)) {
            is_node1_valid = false;
        }
    }
    if (!is_node1_valid && node_ptr1->layer_id == node_ptr2->layer_id) return false;
    // check node1 -> node2
    bool is_node2_valid = true;
    if (node_ptr2->free_direct != NodeFreeDirect::PILLAR) {
        Point3D diff_2to1 = (node_ptr1->position - node_ptr2->position);
        if (!DPUtil::IsInSurfacePairs(diff_2to1, node_ptr2->surf_dirs)) {
            is_node2_valid = false;
        }
    }
    if (!is_node2_valid && node_ptr1->layer_id == node_ptr2->layer_id) return false;
    if (node_ptr1->layer_id != node_ptr2->layer_id && !is_node1_valid && !is_node2_valid) return false;
    return true;
}

bool DynamicGraph::IsInContourDirConstraint(const NavNodePtr& node_ptr1,
                                            const NavNodePtr& node_ptr2) 
{
    if (DPUtil::IsFreeNavNode(node_ptr1) || DPUtil::IsFreeNavNode(node_ptr2)) return false;
    // check node1 -> node2
    if (node_ptr1->is_finalized && node_ptr1->free_direct != NodeFreeDirect::PILLAR) {
        const Point3D diff_1to2 = (node_ptr2->position - node_ptr1->position);
        if (!DPUtil::IsInContourDirPairs(diff_1to2, node_ptr1->surf_dirs)) {
            if (node_ptr1->contour_connects.size() < 2) {
                this->ResetNodeFilters(node_ptr1);
            } 
            return false;
        }
    }
    // check node2 -> node1
    if (node_ptr2->is_finalized && node_ptr2->free_direct != NodeFreeDirect::PILLAR) {
        const Point3D diff_2to1 = (node_ptr1->position - node_ptr2->position);
        if (!DPUtil::IsInContourDirPairs(diff_2to1, node_ptr2->surf_dirs)) {
            if (node_ptr2->contour_connects.size() < 2) {
                this->ResetNodeFilters(node_ptr2);
            }
            return false;
        }
    }
    return true;
}

bool DynamicGraph::IsAShorterConnectInDir(const NavNodePtr& node_ptr_from, const NavNodePtr& node_ptr_to) {
    bool is_nav_connect = false;
    bool is_cover_connect = false;
    if (node_ptr_from->is_navpoint && node_ptr_to->is_navpoint) is_nav_connect = true;
    if (!node_ptr_from->is_frontier && !node_ptr_to->is_frontier) is_cover_connect = true;
    if (node_ptr_from->connect_nodes.empty()) return false;
    Point3D ref_dir, ref_diff;
    const Point3D diff_p = node_ptr_to->position - node_ptr_from->position;
    // ROS_WARN("DG: IsAShorterConnectInDir: %f", diff_p.norm());
    const Point3D connect_dir = diff_p.normalize();
    const float dist = diff_p.norm();
    for (const auto& cnode : node_ptr_from->connect_nodes) {
        if (is_nav_connect && !cnode->is_navpoint) continue;
        if (is_cover_connect && cnode->is_frontier) continue;
        if (DPUtil::IsTypeInStack(cnode, node_ptr_from->contour_connects)) continue;
        ref_diff = cnode->position - node_ptr_from->position;
        if (cnode->is_odom || cnode->is_merged || ref_diff.norm() < DPUtil::kEpsilon) continue;
        ref_dir = ref_diff.normalize();
        if ((connect_dir * ref_dir) > CONNECT_ANGLE_COS && dist > ref_diff.norm()) {
            return true;
        }
    }
    return false;
}

bool DynamicGraph::UpdateNodePosition(const NavNodePtr& node_ptr,
                                      const Point3D& new_pos) 
{
    if (DPUtil::IsFreeNavNode(node_ptr)) {
        InitNodePosition(node_ptr, new_pos);
        return true;
    }
    if (node_ptr->is_finalized) return true; // finalized node 
    node_ptr->pos_filter_vec.push_back(new_pos);
    if (node_ptr->pos_filter_vec.size() > dg_params_.pool_size) {
        node_ptr->pos_filter_vec.pop_front();
    }
    // calculate mean nav node position using RANSACS
    std::size_t inlier_size = 0;
    const Point3D mean_p = DPUtil::RANSACPoisiton(node_ptr->pos_filter_vec, dg_params_.filter_pos_margin, inlier_size);
    node_ptr->position = mean_p;
    if (inlier_size > dg_params_.finalize_thred) {
        return true;
    }
    return false;
}

void DynamicGraph::InitNodePosition(const NavNodePtr& node_ptr, const Point3D& new_pos) {
    node_ptr->pos_filter_vec.clear();
    node_ptr->position = new_pos;
    node_ptr->pos_filter_vec.push_back(new_pos);
}

bool DynamicGraph::UpdateNodeSurfDirs(const NavNodePtr& node_ptr, PointPair cur_dirs)
{
    if (DPUtil::IsFreeNavNode(node_ptr)) {
        node_ptr->surf_dirs = {Point3D(0,0,-1), Point3D(0,0,-1)};
        node_ptr->free_direct = NodeFreeDirect::PILLAR;
        return true;
    }
    if (node_ptr->is_finalized) return true; // finalized node 
    DPUtil::CorrectDirectOrder(node_ptr->surf_dirs, cur_dirs);
    node_ptr->surf_dirs_vec.push_back(cur_dirs);
    if (node_ptr->surf_dirs_vec.size() > dg_params_.pool_size) {
        node_ptr->surf_dirs_vec.pop_front();
    }
    // calculate mean surface corner direction using RANSACS
    std::size_t inlier_size = 0;
    const PointPair mean_dir = DPUtil::RANSACSurfDirs(node_ptr->surf_dirs_vec, dg_params_.filter_dirs_margin, inlier_size);
    if (mean_dir.first == Point3D(0,0,-1) || mean_dir.second == Point3D(0,0,-1)) {
        node_ptr->surf_dirs = {Point3D(0,0,-1), Point3D(0,0,-1)};
        node_ptr->free_direct = NodeFreeDirect::PILLAR;
    } else {
        node_ptr->surf_dirs = mean_dir;
        this->ReEvaluateConvexity(node_ptr);
    }
    if (inlier_size > dg_params_.finalize_thred) {
        return true;
    }
    return false;       
}

void DynamicGraph::ReEvaluateConvexity(const NavNodePtr& node_ptr) {
    // ROS_WARN("DG: Re-evaluate convexity");
    if (!node_ptr->is_contour_match || node_ptr->ctnode->poly_ptr->is_pillar || node_ptr->is_top_layer==true || node_ptr->free_direct == NodeFreeDirect::INSERT) return;
    const Point3D topo_dir = DPUtil::SurfTopoDirect(node_ptr->surf_dirs);
    if (topo_dir.norm() > DPUtil::kEpsilon) {
        const Point3D ctnode_p = node_ptr->ctnode->position;
        const Point3D ev_p = ctnode_p + topo_dir * DPUtil::kLeafSize;
        if (DPUtil::IsConvexPointFromFree(node_ptr->ctnode->poly_ptr->vertices, ev_p, DPUtil::free_odom_p)) {
            node_ptr->free_direct = NodeFreeDirect::CONVEX;
        } else {
            node_ptr->free_direct = NodeFreeDirect::CONCAVE;
        }
    }
}

void DynamicGraph::RecordContourEdge(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
    if (node_ptr1 == node_ptr2) return;
    const auto it1 = node_ptr1->contour_votes.find(node_ptr2->id);
    const auto it2 = node_ptr2->contour_votes.find(node_ptr1->id);
    // DEBUG
    if ((it1 == node_ptr1->contour_votes.end()) != (it2 == node_ptr2->contour_votes.end())) {
        ROS_ERROR_THROTTLE(1.0, "DG: Critical! Contour edge votes map is wrong.");
    }
    if (it1 == node_ptr1->contour_votes.end() || it2 == node_ptr2->contour_votes.end()) {
        // init contour connection votes
        std::deque<int> vote_queue1, vote_queue2;
        vote_queue1.push_back(1), vote_queue2.push_back(1);
        node_ptr1->contour_votes.insert({node_ptr2->id, vote_queue1});
        node_ptr2->contour_votes.insert({node_ptr1->id, vote_queue2});
        if (!DPUtil::IsTypeInStack(node_ptr1, node_ptr2->potential_contours) && !DPUtil::IsTypeInStack(node_ptr2, node_ptr1->potential_contours)) {
            node_ptr1->potential_contours.push_back(node_ptr2);
            node_ptr2->potential_contours.push_back(node_ptr1);
        }
    } else {
        // DEBUG
        if (it1->second.size() != it2->second.size()) ROS_ERROR_THROTTLE(1.0, "DG: contour connection votes are not equal.");
        it1->second.push_back(1), it2->second.push_back(1);
        if (it1->second.size() > dg_params_.votes_size) {
            it1->second.pop_front(), it2->second.pop_front();
        }
        if (it1->second.size() > 2 && DPUtil::IsVoteTrue(it1->second)) {
            // DEBUG
            if (!DPUtil::IsVoteTrue(it2->second)) ROS_ERROR_THROTTLE(1.0, "DG: contour connection vote result are not matched.");
            if (!DPUtil::IsTypeInStack(node_ptr1, node_ptr2->contour_connects) &&
                !DPUtil::IsTypeInStack(node_ptr2, node_ptr1->contour_connects))
            {
                // connect contours
                node_ptr1->contour_connects.push_back(node_ptr2);
                node_ptr2->contour_connects.push_back(node_ptr1);
            }
        }
    }
    // if (node_ptr1->is_wall_insert || node_ptr2->is_wall_insert) {
    //     // ROS_ERROR("==================================");
    //     // for (auto it = node_ptr1->contour_votes.begin(); it != node_ptr1->contour_votes.end(); ++it) {
    //     //     cout<<it->first<<endl;
    //     // }
    //     // ROS_ERROR("==================================");
    //     // for (auto it = node_ptr2->contour_votes.begin(); it != node_ptr2->contour_votes.end(); ++it) {
    //     //     cout<<it->first<<endl;
    //     // }
    //     // ROS_ERROR("==================================");
    //     // ROS_ERROR_STREAM("SS" << node_ptr1->id << " " << node_ptr2->id);
    //     node_ptr1->contour_connects.push_back(node_ptr2);
    //     node_ptr2->contour_connects.push_back(node_ptr1);
    // }
}

void DynamicGraph::RecordPolygonEdge(const NavNodePtr& node_ptr1, 
                                     const NavNodePtr& node_ptr2,
                                     const int& queue_size, 
                                     const bool& is_reset) 
{
    if (node_ptr1 == node_ptr2) return;
    const auto it1 = node_ptr1->edge_votes.find(node_ptr2->id);
    const auto it2 = node_ptr2->edge_votes.find(node_ptr1->id);
    // DEBUG
    if ((it1 == node_ptr1->edge_votes.end()) != (it2 == node_ptr2->edge_votes.end())) {
        ROS_ERROR_THROTTLE(1.0, "DG: Critical! Polygon edge votes map is wrong.");
    }
    if (it1 == node_ptr1->edge_votes.end() || it2 == node_ptr2->edge_votes.end()) {
        // init polygon edge votes
        std::deque<int> vote_queue1, vote_queue2;
        vote_queue1.push_back(1), vote_queue2.push_back(1);
        node_ptr1->edge_votes.insert({node_ptr2->id, vote_queue1});
        node_ptr2->edge_votes.insert({node_ptr1->id, vote_queue2});
        if (!DPUtil::IsTypeInStack(node_ptr1, node_ptr2->potential_edges) && !DPUtil::IsTypeInStack(node_ptr2, node_ptr1->potential_edges)) {
            node_ptr1->potential_edges.push_back(node_ptr2);
            node_ptr2->potential_edges.push_back(node_ptr1);
        }
    } else {
        if (it1->second.size() != it2->second.size()) ROS_ERROR_THROTTLE(1.0, "DG: Polygon edge votes are not equal.");
        if (is_reset) it1->second.clear(), it2->second.clear();
        it1->second.push_back(1), it2->second.push_back(1);
        if (it1->second.size() > queue_size) {
            it1->second.pop_front(), it2->second.pop_front();
        }
    }
}

void DynamicGraph::DeletePolygonEdge(const NavNodePtr& node_ptr1, 
                                     const NavNodePtr& node_ptr2,
                                     const int& queue_size,
                                     const bool& is_reset) 
{
    const auto it1 = node_ptr1->edge_votes.find(node_ptr2->id);
    const auto it2 = node_ptr2->edge_votes.find(node_ptr1->id);
    if (it1 == node_ptr1->edge_votes.end() || it2 == node_ptr2->edge_votes.end()) return;
    if (is_reset) it1->second.clear(), it2->second.clear();
    it1->second.push_back(0), it2->second.push_back(0);
    if (it1->second.size() > queue_size) {
        it1->second.pop_front(), it2->second.pop_front();
    }
}

/* Delete Contour edge for given two navigation nodes */
void DynamicGraph::DeleteContourEdge(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
    const auto it1 = node_ptr1->contour_votes.find(node_ptr2->id);
    const auto it2 = node_ptr2->contour_votes.find(node_ptr1->id);
    if (it1 == node_ptr1->contour_votes.end() || it2 == node_ptr2->contour_votes.end()) return; // no connection (not counter init) in the first place 
    it1->second.push_back(0), it2->second.push_back(0);
    if (it1->second.size() > dg_params_.votes_size) {
        it1->second.pop_front(), it2->second.pop_front();
    }
    if (!DPUtil::IsVoteTrue(it1->second)) {
        // clear node2 in node1's connection
        DPUtil::EraseNodeFromStack(node_ptr2, node_ptr1->contour_connects);
        DPUtil::EraseNodeFromStack(node_ptr1, node_ptr2->contour_connects);
        // clear connect counters
        if (it1->second.size() == dg_params_.votes_size && std::accumulate(it1->second.begin(), it1->second.end(), 0) < 1) {
            node_ptr1->contour_votes.erase(node_ptr2->id);
            node_ptr2->contour_votes.erase(node_ptr1->id);
            // clear potential connections
            DPUtil::EraseNodeFromStack(node_ptr2, node_ptr1->potential_contours);
            DPUtil::EraseNodeFromStack(node_ptr1, node_ptr2->potential_contours);
        }
    }
}

void DynamicGraph::UpdateGlobalNearNodes() {
    /* update nearby navigation nodes stack --> near_nav_nodes_ */
    near_nav_nodes_.clear(), wide_near_nodes_.clear(), internav_near_nodes_.clear(), surround_internav_nodes_.clear();
    for (const auto& node_ptr : globalGraphNodes_) {
        node_ptr->is_near_nodes = false;
        node_ptr->is_wide_near = false;
        const float dist = (node_ptr->position - odom_node_ptr_->position).norm();
        if (dist < 3.0 * dg_params_.sensor_range && DPUtil::IsTypeInStack(node_ptr->layer_id, wide_layer_idxs_)) {
            if (node_ptr->is_goal) continue;
            wide_near_nodes_.push_back(node_ptr);
            node_ptr->is_wide_near = true;
            if (dist < dg_params_.sensor_range && DPUtil::IsTypeInStack(node_ptr->layer_id, cur_layer_idxs_)) {
                near_nav_nodes_.push_back(node_ptr);
                node_ptr->is_near_nodes = true;
                if (node_ptr->is_navpoint) {
                    node_ptr->position.intensity = node_ptr->fgscore;
                    internav_near_nodes_.push_back(node_ptr);
                    if (node_ptr->gscore < DPUtil::kNearDist * 2.0) {
                        surround_internav_nodes_.push_back(node_ptr);
                    }
                }
            }
        }
    }
    for (const auto& cnode_ptr : odom_node_ptr_->connect_nodes) { // add additional odom connections to wide near stack
        if (!cnode_ptr->is_wide_near && !cnode_ptr->is_goal) {
            wide_near_nodes_.push_back(cnode_ptr);
            cnode_ptr->is_wide_near = true;
        }
    }
    if (!internav_near_nodes_.empty()) { // find the nearest inter_nav node that connect to odom
        std::sort(internav_near_nodes_.begin(), internav_near_nodes_.end(), nodeptr_icomp());
        for (std::size_t i=0; i<internav_near_nodes_.size(); i++) {
            const NavNodePtr temp_internav_ptr = internav_near_nodes_[i];
            if (DPUtil::IsTypeInStack(temp_internav_ptr, odom_node_ptr_->potential_edges) && this->IsInternavInRange(temp_internav_ptr)) {
                if (cur_internav_ptr_ == NULL || temp_internav_ptr == cur_internav_ptr_ || (temp_internav_ptr->position - cur_internav_ptr_->position).norm() < DPUtil::kNearDist ||
                    DPUtil::IsTypeInStack(temp_internav_ptr, cur_internav_ptr_->connect_nodes)) 
                {   
                    this->UpdateCurInterNavNode(temp_internav_ptr);  
                } else {
                    if (DPUtil::IsTrajectory) is_bridge_internav_ = true;
                    else this->UpdateCurInterNavNode(temp_internav_ptr);
                }
                break;
            }
        }
    }
}

bool DynamicGraph::ReEvaluateCorner(const NavNodePtr node_ptr) {
    if (DPUtil::IsPointNearNewPoints(node_ptr->position, false)) {
        this->ResetNodeFilters(node_ptr);
        this->ResetNodeConnectVotes(node_ptr);
    }
    if (node_ptr->is_finalized) {
        // if (node_ptr->connect_nodes.empty() && node_ptr->is_wall_insert) cout<<"wall insert node has no connections"<<endl;
        if (node_ptr->connect_nodes.empty() && !node_ptr->is_wall_insert) return false;
        else if (!node_ptr->is_contour_match && !node_ptr->is_wall_insert) {
            if (DPUtil::IsOutReachNode(node_ptr)) return true;
            return false; 
        } 
        else return true;
        // return true;
    }

    if (!node_ptr->is_contour_match) return false;
    // if nearby env changes;
    const bool is_pos_cov  = this->UpdateNodePosition(node_ptr, node_ptr->ctnode->position);
    const bool is_dirs_cov = this->UpdateNodeSurfDirs(node_ptr, node_ptr->ctnode->surf_dirs);
    ROS_ERROR_COND(node_ptr->free_direct == NodeFreeDirect::UNKNOW, "DG: node free space is unknown.");
    // detect wall
    if (node_ptr->free_direct != NodeFreeDirect::PILLAR) {
        const float dot_value = node_ptr->surf_dirs.first * node_ptr->surf_dirs.second;
        if (dot_value < ALIGN_ANGLE_COS && !node_ptr->is_wall_insert) { // wall detected
            return false;
        }
    }
    if (is_pos_cov && is_dirs_cov) {
        node_ptr->is_finalized = true;
    } 
    return true;
}

bool DynamicGraph::ReEvaluateConnectUsingTerrian(const NavNodePtr& node_ptr1, const NavNodePtr node_ptr2) {
    PointStack terrain_path;
    if (terrain_planner_.PlanPathFromNodeToNode(node_ptr1, node_ptr2, terrain_path)) {
        return true;
    }
    return false;
}

void DynamicGraph::ConnectVerticalNodes(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
    if (std::abs(node_ptr1->layer_id - node_ptr2->layer_id) != 1) return;
    if (node_ptr1->ctnode != NULL && node_ptr2->ctnode != NULL) {
        if (node_ptr1->ctnode->up != NULL && node_ptr2->ctnode->down != NULL) {
            if ((node_ptr1->ctnode->up->position - node_ptr2->ctnode->position).norm() < DPUtil::kNearDist / 3 || 
            (node_ptr2->ctnode->down->position - node_ptr1->ctnode->position).norm() < DPUtil::kNearDist / 3) {
                node_ptr1->contour_connects.push_back(node_ptr2);
                node_ptr2->contour_connects.push_back(node_ptr1);
                node_ptr1->up_node = node_ptr2;
                node_ptr2->down_node = node_ptr1;
                return;
            }
        }
        if (node_ptr1->ctnode->down != NULL && node_ptr2->ctnode->up != NULL) {
            if ((node_ptr1->ctnode->down->position - node_ptr2->ctnode->position).norm() < DPUtil::kNearDist / 3 || 
            (node_ptr2->ctnode->up->position - node_ptr1->ctnode->position).norm() < DPUtil::kNearDist / 3) {
                node_ptr1->contour_connects.push_back(node_ptr2);
                node_ptr2->contour_connects.push_back(node_ptr1);
                node_ptr1->down_node = node_ptr2;
                node_ptr2->up_node = node_ptr1;
            }
        }
    }
}

void DynamicGraph::AddVerticalEdges(const NavNodePtr& node_ptr, const NavNodePtr& connected_ptr) {
    // if (connected_ptr->up_node == NULL) return;

    // check upward
    this->AddUpwardEdges(node_ptr, connected_ptr);

    // check downward
    this->AddDownwardEdges(node_ptr, connected_ptr);
}

void DynamicGraph::AddUpwardEdges(const NavNodePtr& node_ptr, const NavNodePtr& connected_ptr) {
    if (connected_ptr->up_node == NULL || node_ptr == NULL) return;

    NavNodePtr up_node_ptr = connected_ptr->up_node;
    if (node_ptr->is_contour_match && up_node_ptr->is_contour_match &&
        !DPUtil::IsTypeInStack(up_node_ptr, node_ptr->connect_nodes)) {              
        if (IsValidConnect(node_ptr, up_node_ptr, true, false, false)) {
            AddEdge(node_ptr, up_node_ptr);
        } else {
            this->EraseEdge(node_ptr, up_node_ptr);
        }
    }
    AddUpwardEdges(node_ptr, up_node_ptr);   
}

void DynamicGraph::AddDownwardEdges(const NavNodePtr& node_ptr, const NavNodePtr& connected_ptr) {
    if (connected_ptr->down_node == NULL || node_ptr == NULL) return;

    NavNodePtr down_node_ptr = connected_ptr->down_node;
    if (node_ptr->is_contour_match && down_node_ptr->is_contour_match &&
        !DPUtil::IsTypeInStack(down_node_ptr, node_ptr->connect_nodes)) {              
        if (IsValidConnect(node_ptr, down_node_ptr, true, false, false)) {
            AddEdge(node_ptr, down_node_ptr);
        } else {
            this->EraseEdge(node_ptr, down_node_ptr);
        }
    }
    AddUpwardEdges(node_ptr, down_node_ptr);
    }

/************************** UNUSE CODE ****************************/

// bool DynamicGraph::IsValidVertConnect(const NavNodePtr& node_ptr1, 
//                                   const NavNodePtr& node_ptr2,
//                                   const bool& is_local_only,
//                                   const bool& is_check_contour,
//                                   bool& _is_merge,
//                                   bool& _is_matched) 
// {
//     if (node_ptr1->is_contour_match && node_ptr2->is_contour_match) {
//         _is_matched = true;
//     } else {
//         _is_matched = false;
//     }
//     if (!DPUtil::IsFreeNavNode(node_ptr1) && !DPUtil::IsFreeNavNode(node_ptr2) && node_ptr1->layer_id == node_ptr2->layer_id) {
//         const float dist = (node_ptr1->position - node_ptr2->position).norm();
//         if (dist < DPUtil::kNavClearDist && node_ptr1->free_direct == node_ptr2->free_direct) {
//             bool is_same_dir = true;
//             if (node_ptr1->free_direct != NodeFreeDirect::PILLAR) {
//                 const Point3D topo_dir1 = DPUtil::SurfTopoDirect(node_ptr1->surf_dirs);
//                 const Point3D topo_dir2 = DPUtil::SurfTopoDirect(node_ptr2->surf_dirs);
//                 if (topo_dir1 * topo_dir2 < NOISE_ANGLE_COS) is_same_dir = false;
//             }
//             if (is_same_dir) {
//                 _is_merge = true;
//                 return false;
//             }
//         }
//     }   
//     _is_merge = false;
//     /* check contour connection from node1 to node2 */
//     bool is_connect = false;
//     if (!is_connect && is_check_contour && node_ptr1->layer_id == node_ptr2->layer_id) {
//         if (ContourGraph::IsNavNodesConnectFromContour(node_ptr1, node_ptr2) && this->IsInContourDirConstraint(node_ptr1, node_ptr2)) {
//             this->RecordContourEdge(node_ptr1, node_ptr2);
//         } else if (node_ptr1->is_contour_match && node_ptr2->is_contour_match) {
//             this->DeleteContourEdge(node_ptr1, node_ptr2);
//         }
//     }
//     if (DPUtil::IsTypeInStack(node_ptr1, node_ptr2->contour_connects)) is_connect = true;
//     /* check if the trajectory connect is recorded but not valid */
//     bool is_invalid_traj = false;
//     if (DPUtil::IsTrajectory && node_ptr1->is_navpoint && node_ptr2->is_navpoint) {
//         const auto it = node_ptr1->trajectory_votes.find(node_ptr2->id);
//         if (it != node_ptr1->trajectory_votes.end() && !DPUtil::IsVoteTrue(it->second)) {
//             is_invalid_traj = true;
//         } 
//     }
//     // if (std::abs(node_ptr1->layer_id - node_ptr2->layer_id)>2) {
//     // cout<<"111Layer id: "<<node_ptr1->layer_id<<endl;
//     // cout<<"222Layer id: "<<node_ptr2->layer_id<<endl;
//     // }

//     /* check polygon connections */
//     if (!is_connect && !is_invalid_traj) {
//         bool is_valid_edge = false;
//         const int vote_queue_size = (node_ptr1->is_odom || node_ptr2->is_odom) ? std::ceil(dg_params_.votes_size/3.0) : dg_params_.votes_size;
//         if ((node_ptr1->free_direct != NodeFreeDirect::CONCAVE || node_ptr2->is_odom) && 
//             (node_ptr2->free_direct != NodeFreeDirect::CONCAVE || node_ptr1->is_odom)) 
//         {
//     //             if (std::abs(node_ptr1->layer_id - node_ptr2->layer_id)>2) {
//     // cout<<"111Layer id: "<<node_ptr1->layer_id<<endl;
//     // cout<<"222Layer id: "<<node_ptr2->layer_id<<endl;
//     // cout<<"is_valid_edge: "<<is_valid_edge<<endl;
//     // }
//             if (this->IsInDirectConstraint(node_ptr1, node_ptr2) && ContourGraph::IsNavNodesConnectFreePolygon(node_ptr1, node_ptr2, is_local_only)) {
//     //                             if (std::abs(node_ptr1->layer_id - node_ptr2->layer_id)>2) {
//     // cout<<"333Layer id: "<<node_ptr1->layer_id<<endl;
//     // cout<<"444Layer id: "<<node_ptr2->layer_id<<endl;
//     // cout<<"is_valid_edge: "<<is_valid_edge<<endl;
//     // }
//                 is_valid_edge = true;
//                 this->RecordPolygonEdge(node_ptr1, node_ptr2, vote_queue_size);
//             }
//         }
//         if (!is_valid_edge) this->DeletePolygonEdge(node_ptr1, node_ptr2, vote_queue_size);
//         if (this->IsPolygonEdgeVoteTrue(node_ptr1, node_ptr2)) {
//     // if (std::abs(node_ptr1->layer_id - node_ptr2->layer_id)>2) {
//     // cout<<"555Layer id: "<<node_ptr1->layer_id<<endl;
//     // cout<<"666Layer id: "<<node_ptr2->layer_id<<endl;
//     // }
//             if (!this->IsSimilarConnectInDiection(node_ptr1, node_ptr2)) {
//                 is_connect = true;
//                 // cout<<"hereeeeeeeee"<<endl;
//             } 
//         } else if (node_ptr1->is_odom || node_ptr2->is_odom) {
//             node_ptr1->edge_votes.erase(node_ptr2->id);
//             node_ptr2->edge_votes.erase(node_ptr1->id);
//             // clear potential connections
//             DPUtil::EraseNodeFromStack(node_ptr2, node_ptr1->potential_edges);
//             DPUtil::EraseNodeFromStack(node_ptr1, node_ptr2->potential_edges);
//         }
//     }
//     // cout<<"333is_connect: "<<is_connect<<endl;
//     /* check if exsiting trajectory connection exist */
//     if (DPUtil::IsTrajectory && !is_connect) {
//         if (DPUtil::IsTypeInStack(node_ptr1, node_ptr2->trajectory_connects)) is_connect = true;
//         if ((node_ptr1->is_odom || node_ptr2->is_odom) && cur_internav_ptr_ != NULL) {
//             if ((odom_node_ptr_->position - cur_internav_ptr_->position).norm() < DPUtil::kNearDist) {
//                 if (node_ptr1->is_odom && DPUtil::IsTypeInStack(node_ptr2, cur_internav_ptr_->trajectory_connects)) {
//                     is_connect = true;
//                 } else if (node_ptr2->is_odom && DPUtil::IsTypeInStack(node_ptr1, cur_internav_ptr_->trajectory_connects)) {
//                     is_connect = true;
//                 }
//             }   
//         }
//     }
//     // cout<<"444is_connect: "<<is_connect<<endl;
//     return is_connect;
// }

// bool DynamicGraph::IsValidVertConnect(const NavNodePtr& node_ptr1, 
//                                   const NavNodePtr& node_ptr2,
//                                   const bool& is_local_only,
//                                   const bool& is_check_contour) 
// {
//     bool UNUSE_is_merge, UNUSE_is_matched;
//     return this->IsValidVertConnect(node_ptr1, node_ptr2, is_local_only, is_check_contour, UNUSE_is_merge, UNUSE_is_matched);
// }