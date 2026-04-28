/*
 * Dynamic Route Planner
 * Copyright (C) 2021 Fan Yang - All rights reserved
 * fanyang2@andrew.cmu.edu,   
 */



#include "airfar_planner/graph_planner.h"
// #include "graph_planner.h"
// #include "graph_planner.h"

/***************************************************************************************/

const char INIT_BIT  = char(0); // 0000
const char OBS_BIT   = char(1); // 0001
const char FREE_BIT  = char(2); // 0010


void GraphPlanner::Init(const ros::NodeHandle& nh, const GraphPlannerParams& params) {
    nh_ = nh;
    gp_params_ = params;
    is_goal_init_ = false;
    current_graph_.clear();
    gp_viz_.Init(nh);
    // attemptable planning listener
    attemptable_sub_ = nh_.subscribe("/planning_attemptable", 5, &GraphPlanner::AttemptStatusCallBack, this);
    planning_time_pub_ = nh_.advertise<std_msgs::Float32>("/far_planning_time", 5);
    traverse_time_pub_ = nh_.advertise<std_msgs::Float32>("/far_traverse_time", 5);
    reach_goal_pub_    = nh_.advertise<std_msgs::Bool>("/far_reach_goal_status", 5);
}

void GraphPlanner::UpdateGraphTraverability(const NavNodePtr& odom_node_ptr) 
{
    if (odom_node_ptr == NULL || global_graph_.empty()) {
        ROS_ERROR("GP: Update global graph traversablity fails.");
        return;
    }
    std::cout<<"graph size: "<<global_graph_.size()<<std::endl;
    odom_node_ptr_ = odom_node_ptr;
    current_graph_ = global_graph_;
    this->InitNodesStates(current_graph_);
    // start expand the whole current_graph_
    odom_node_ptr_->gscore = 0.0;
    std::unordered_set<std::size_t> open_set;
    std::priority_queue<NavNodePtr, NodePtrStack, nodeptr_gcomp> open_queue;
    std::unordered_set<NavNodePtr, nodeptr_hash, nodeptr_equal> close_set;
    // Expansion from odom node to all reachable navigation node
    open_queue.push(odom_node_ptr_);
    open_set.insert(odom_node_ptr_->id);
    
    std_msgs::Float32 planning_time;
    DPUtil::Timer.start_time("Path_Searching");
    reached_goal_msg_.data = false;

    while (!open_set.empty()) {
        const NavNodePtr current = open_queue.top();
        open_queue.pop();
        open_set.erase(current->id);
        close_set.insert(current);
        current->is_traversable = true; // reachable from current position
        // std::cout<<"current connect_nodes size: "<<current->connect_nodes.size()<<std::endl;
        for (const auto& neighbor : current->connect_nodes) {
            if (close_set.count(neighbor)) continue;
            // if (current->is_inserted) ROS_ERROR("GP: inserted node in open set");
            const float temp_gscore = current->gscore + this->EulerCost(current, neighbor);
            if (temp_gscore < neighbor->gscore) {
                neighbor->parent = current;
                neighbor->gscore = temp_gscore;
                if (!open_set.count(neighbor->id)) {
                    open_queue.push(neighbor);
                    open_set.insert(neighbor->id);
                }
            }
        }
    }
    // cout<<"GP: Graph traversability updated."<<endl;

    planning_time.data = DPUtil::Timer.end_time("Path_Searching");
    planning_time_pub_.publish(planning_time);
    reach_goal_pub_.publish(reached_goal_msg_);

    std::priority_queue<NavNodePtr, NodePtrStack, nodeptr_fgcomp> fopen_queue;
    std::unordered_set<std::size_t> fopen_set;
    close_set.clear();
    // Expansion from odom node to all covered navigation node
    odom_node_ptr_->fgscore = 0.0;
    fopen_queue.push(odom_node_ptr_);
    fopen_set.insert(odom_node_ptr_->id);
    while (!fopen_set.empty()) {
        const NavNodePtr current = fopen_queue.top();
        fopen_queue.pop();
        fopen_set.erase(current->id);
        close_set.insert(current);
        current->is_free_traversable = true; // reachable from current position
        for (const auto& neighbor : current->connect_nodes) {
            if (neighbor->is_frontier || close_set.count(neighbor)) continue;
            const float e_dist = this->EulerCost(current, neighbor);
            if (neighbor == goal_node_ptr_ && e_dist > DPUtil::kSensorRange) continue; // To goal distance should less than sensor range
            const float temp_fgscore = current->fgscore + e_dist;
            if (temp_fgscore < neighbor->fgscore) {
                neighbor->free_parent = current;
                neighbor->fgscore = temp_fgscore;
                if (!fopen_set.count(neighbor->id)) {
                    fopen_queue.push(neighbor);
                    fopen_set.insert(neighbor->id);
                } 
            }
        }
    }
}

void GraphPlanner::UpdateGoalNavNodeConnects(const NavNodePtr& goal_node_ptr)
{
    if (goal_node_ptr_ == NULL || is_use_internav_goal_) return;
    this->ReEvaluateGoalStatus(goal_node_ptr_, global_graph_);
    for (const auto& node_ptr : global_graph_) {
        if (node_ptr == goal_node_ptr_) continue;
        // ROS_WARN("GP: Update goal node connect status: %d", this->IsValidConnectToGoal(node_ptr, goal_node_ptr_));
        if (this->IsValidConnectToGoal(node_ptr, goal_node_ptr_)) {
            const bool is_directly_connect = node_ptr->is_odom ? true : false;
            DynamicGraph::RecordPolygonEdge(node_ptr, goal_node_ptr_, gp_params_.votes_size, is_directly_connect);
        } else {
            DynamicGraph::DeletePolygonEdge(node_ptr, goal_node_ptr_, gp_params_.votes_size);
        }
        const auto it = goal_node_ptr_->edge_votes.find(node_ptr->id);
        if (it != goal_node_ptr_->edge_votes.end() && DPUtil::IsVoteTrue(it->second)) {
            DynamicGraph::AddEdge(node_ptr, goal_node_ptr_);
            node_ptr->is_block_to_goal = false;
            node_ptr->is_reach_goal = true;
        } else {
            DynamicGraph::EraseEdge(node_ptr, goal_node_ptr_);
            node_ptr->is_block_to_goal = true;
            node_ptr->is_reach_goal = false;
        }
    }
}

bool GraphPlanner::IsValidConnectToOdom(const NavNodePtr& node_ptr, const NavNodePtr& odom_node_ptr) {
    // ROS_WARN("GP: Is valid connect to odom");
    // cout<<DynamicGraph::IsConnectInVerticalConstrain(node_ptr, odom_node_ptr)<<endl;
    // cout<<ContourGraph::IsNavToOdomConnectFreePolygon(node_ptr, odom_node_ptr)<<endl;
    // ROS_WARN("=================================");
    if (DynamicGraph::IsConnectInVerticalConstrain(node_ptr, odom_node_ptr) && 
        ContourGraph::IsNavToOdomConnectFreePolygon(node_ptr, odom_node_ptr))
    {
        return true;
    }
    return false;
}

bool GraphPlanner::IsValidConnectToGoal(const NavNodePtr& node_ptr, const NavNodePtr& goal_node_ptr) {
    // ROS_ERROR("GP: Is valid connect to goal");
    // cout<<DynamicGraph::IsConnectInVerticalConstrain(node_ptr, goal_node_ptr)<<endl;
    // cout<<ContourGraph::IsNavToGoalConnectFreePolygon(node_ptr, goal_node_ptr)<<endl;
    // ROS_ERROR("=================================");
    if (!node_ptr->is_block_to_goal || IsResetBlockStatus(node_ptr)) {
        if (DynamicGraph::IsConnectInVerticalConstrain(node_ptr, goal_node_ptr) && 
            ContourGraph::IsNavToGoalConnectFreePolygon(node_ptr, goal_node_ptr))
        {
            return true;
        }
    }
    return false;
}

bool GraphPlanner::NextGoalPlanning(PointStack& global_path,
                                    Point3D& _nav_goal,
                                    Point3D& _goal_p,
                                    bool& _is_fails,
                                    bool& _is_free_nav,
                                    NodePtrStack& global_path_ptr) 
{
    if (!is_goal_init_) return false;

    std_msgs::Float32 traverse_timer;
    traverse_timer.data = DPUtil::Timer.record_time("Overall_executing");
    traverse_time_pub_.publish(traverse_timer);

    if (odom_node_ptr_ == NULL || goal_node_ptr_ == NULL || current_graph_.empty()) {
        ROS_ERROR("GP: Graph or Goal is not initialized correctly.");
        return false;
    }
    _is_fails = false;
    global_path.clear();
    global_path_ptr.clear();
    
    // _goal_p = goal_node_ptr_->position;
    _goal_p = origin_goal_pos_;
    // ROS_WARN("original goal position: %f, %f, %f", _goal_p.x, _goal_p.y, _goal_p.z);
    // ROS_WARN("goal position: %f, %f, %f", goal_node_ptr_->position.x, goal_node_ptr_->position.y, goal_node_ptr_->position.z);
    if (current_graph_.size() == 1) {
        const Point3D diff_p = (_goal_p - odom_node_ptr_->position).normalize();
        const Point3D next_goal = odom_node_ptr_->position + diff_p * 5.0;
        // update global path
        global_path.push_back(odom_node_ptr_->position);
        global_path.push_back(next_goal);
        global_path_ptr.push_back(odom_node_ptr_);
        _nav_goal = this->NextNavWaypointFromPath(global_path);
        _is_free_nav = is_free_nav_goal_;
        return true;       
    }
    if ((odom_node_ptr_->position - _goal_p). norm() < gp_params_.converge_dist || 
        (odom_node_ptr_->position - origin_goal_pos_).norm() < gp_params_.converge_dist)
    {
        ROS_INFO("GP: *********** Goal Reached! ***********");

        reached_goal_msg_.data = true;
        traverse_timer.data = DPUtil::Timer.end_time("Overall_executing");
        traverse_time_pub_.publish(traverse_timer);
        reach_goal_pub_.publish(reached_goal_msg_);

        global_path.push_back(odom_node_ptr_->position);
        global_path_ptr.push_back(odom_node_ptr_);
        if ((odom_node_ptr_->position - _goal_p). norm() > gp_params_.converge_dist) _goal_p = origin_goal_pos_;
        global_path.push_back(_goal_p);
        global_path_ptr.push_back(goal_node_ptr_);
        _nav_goal = _goal_p;
        _is_free_nav = is_free_nav_goal_;
        this->GoalReset();
        is_goal_init_ = false;
        return true;
    }
    // check free space navigation status
    if (gp_params_.is_autoswitch && command_is_free_nav_) {
        if (goal_node_ptr_->is_free_traversable || (is_free_nav_goal_ && is_global_path_init_ && path_momentum_counter_ < gp_params_.momentum_thred)) {
            is_free_nav_goal_ = true;
        } else {
            is_free_nav_goal_ = false;
        }
    }   
    _is_free_nav = is_free_nav_goal_;
    const NavNodePtr reach_nav_node = is_free_nav_goal_ ? goal_node_ptr_->free_parent : goal_node_ptr_->parent;
    if (is_global_path_init_ && last_waypoint_dist_ > gp_params_.adjust_radius && reach_nav_node != NULL && reach_nav_node != odom_node_ptr_) {
        // momentum planning if movement towards to current waypoints is less than threshold
        const float cur_waypoint_dist = (odom_node_ptr_->position - next_waypoint_).norm();
        if (cur_waypoint_dist > gp_params_.converge_dist && (abs((odom_node_ptr_->position - last_planning_odom_).norm() < gp_params_.momentum_dist) 
            && abs(cur_waypoint_dist - last_waypoint_dist_) < gp_params_.momentum_dist)) 
        {
            ROS_INFO("GP: inside momentum distance, follow the previous waypoint.");
            recorded_path_[0] = odom_node_ptr_->position;
            global_path = recorded_path_;
            _nav_goal = this->NextNavWaypointFromPath(global_path);
            return true;
        }
    }
    if (reach_nav_node == NULL) {
        // no reachable goal found
        if (is_global_path_init_ && path_momentum_counter_ < gp_params_.momentum_thred) {
            ROS_WARN("GP: no reachable nav node found, path momentum planning...");
            global_path = recorded_path_;
            _nav_goal = this->NextNavWaypointFromPath(global_path);
            if (!odom_node_ptr_->connect_nodes.empty()) {
                path_momentum_counter_ ++;
            }
            return true;
        } else {
            if (gp_params_.is_autoswitch && is_free_nav_goal_) {
                ROS_WARN("GP: free navigation fails, auto swiching to attemptable navigation...");
                if (is_global_path_init_) {
                    global_path = recorded_path_;
                    _nav_goal = this->NextNavWaypointFromPath(global_path);
                } else {
                    _nav_goal = _goal_p;
                }
                is_free_nav_goal_ = false;
                return true;
            }
            ROS_ERROR("****************** FAIL TO REACH GOAL ******************");
            this->GoalReset();
            is_goal_init_ = false, _is_fails = true;
            return false;
        }
    }
    if (this->ReconstructPath(goal_node_ptr_, is_free_nav_goal_, global_path, global_path_ptr)) {
        // recheck path
        // this->recheckPath(odom_node_ptr_, global_path, global_path_ptr);
        if (!is_divided_path) {
            NodePtrStack divided_path_;
            GetDividedPath(global_path_ptr, divided_path_);
            // cout << "GP: global path size: " << global_path_ptr.size() << endl;
            // cout << "GP: divided path size: " << divided_path_.size() << endl;  
            is_divided_path = true;          
        } else {
            // ROS_WARN("GP: global path is already divided, global path size: %d", global_path_ptr.size());
        }
        _nav_goal = this->NextNavWaypointFromPath(global_path);
        this->RecordPathInfo(global_path);
        return true;
    }
    this->GoalReset();
    is_goal_init_ = false, _is_fails = true;
    return false;
}

bool GraphPlanner::ReconstructPath(const NavNodePtr& goal_node_ptr,
                                   const bool& is_free_nav,
                                   PointStack& global_path,
                                   NodePtrStack& global_path_ptr)
{
    if (goal_node_ptr == NULL || (!is_free_nav && goal_node_ptr->parent == NULL) || (is_free_nav && goal_node_ptr->free_parent == NULL)) {
        ROS_ERROR("GP: reconstruct path error: goal node or its parent equals to NULL.");
        return false;
    }
    global_path.clear();
    global_path_ptr.clear();
    NavNodePtr check_ptr = goal_node_ptr;
    global_path.push_back(check_ptr->position);
    global_path_ptr.push_back(check_ptr);
    if (is_free_nav) {
        while (true) {
            const NavNodePtr parent_ptr = check_ptr->free_parent;
            global_path.push_back(parent_ptr->position);
            global_path_ptr.push_back(parent_ptr);
            if (parent_ptr->free_parent == NULL) break;
            check_ptr = parent_ptr;
        }
    } else {
        while (true) {
            const NavNodePtr parent_ptr = check_ptr->parent;
            global_path.push_back(parent_ptr->position);
            global_path_ptr.push_back(parent_ptr);
            if (parent_ptr->parent == NULL) break;
            check_ptr = parent_ptr;
        } 
    }
    std::reverse(global_path.begin(), global_path.end()); 
    std::reverse(global_path_ptr.begin(), global_path_ptr.end());

    return true;
}

void GraphPlanner::recheckPath(const NavNodePtr& odom_node_ptr, PointStack& global_path, NodePtrStack& global_path_ptr) {
    if (global_path.size() < 2) {
        ROS_ERROR("GP: global path size less than 2.");
        return;
    }

    const std::size_t path_size = global_path.size();
    std::size_t nav_idx = 1;

    for (std::size_t i = 1; i < path_size; i++) {
        if (this->IsValidConnectToOdom(global_path_ptr[i], odom_node_ptr)) {
            DynamicGraph::AddEdge(global_path_ptr[i], odom_node_ptr);
            nav_idx = i;
        } else {
            break;
        }
    }

    if (nav_idx > 1 && nav_idx < path_size - 1) {
        ROS_ERROR("GP: recheck path, remove nodes from %d to %d", 1, nav_idx - 1);
        if (nav_idx == 2) {
            global_path.erase(global_path.begin() + 1);
            global_path_ptr.erase(global_path_ptr.begin() + 1);
        } else {
            global_path.erase(global_path.begin() + 1, global_path.begin() + nav_idx - 1);
            global_path_ptr.erase(global_path_ptr.begin() + 1, global_path_ptr.begin() + nav_idx - 1);
        }
    }
}

Point3D GraphPlanner::NextNavWaypointFromPath(const PointStack& global_path) {
    if (global_path.size() < 2) {
        ROS_ERROR("GP: global path size less than 2.");
        return goal_node_ptr_->position;
    }
    Point3D nav_waypoint;
    const std::size_t path_size = global_path.size();
    std::size_t nav_idx = 1;
    nav_waypoint = global_path[nav_idx];
    float dist = (nav_waypoint - odom_node_ptr_->position).norm();
    while (dist < gp_params_.converge_dist) {
        nav_idx ++;
        if (nav_idx < path_size) {
            nav_waypoint = global_path[nav_idx];
            dist = (nav_waypoint - odom_node_ptr_->position).norm();
        } else break;
    }
    return nav_waypoint;
}

void GraphPlanner::UpdateGoal(const Point3D& goal, const int& layer_id, const bool& is_free_nav) {
    this->GoalReset();
    is_use_internav_goal_ = false;
    float min_dist = DPUtil::kNearDist;
    for (const auto& node_ptr : current_graph_) {
        node_ptr->is_block_to_goal = false;
        if (DPUtil::IsNavpoint && node_ptr->is_navpoint) { // check if goal is near internav node
            const float cur_dist = (node_ptr->position - goal).norm();
            if (cur_dist < min_dist) {
                is_use_internav_goal_ = true;
                goal_node_ptr_ = node_ptr;
                min_dist = cur_dist;
            }
        }
    }
    if (!is_use_internav_goal_) {
        DynamicGraph::CreateNavNodeFromPoint(goal, layer_id, goal_node_ptr_, false, false, true);
        DynamicGraph::AddNodeToGraph(goal_node_ptr_);

    // ROS_WARN("22222goal position: %f, %f, %f", goal_node_ptr_->position.x, goal_node_ptr_->position.y, goal_node_ptr_->position.z);
    }
    ROS_INFO("GP: *********** new goal updated ***********");
    is_goal_init_ = true;
    is_global_path_init_ = false;
    origin_goal_pos_ = goal_node_ptr_->position;
    origin_layer_id_ = layer_id;
    is_origin_free_ = is_use_internav_goal_ ? true : false;
    command_is_free_nav_ = is_free_nav;
    is_free_nav_goal_ = command_is_free_nav_;
    next_waypoint_ = Point3D(0,0,0);
    last_waypoint_dist_ = 0.0;
    last_planning_odom_ = Point3D(0,0,0);
    recorded_path_.clear();
    path_momentum_counter_ = 0;
    reached_goal_msg_.data = false;
    DPUtil::Timer.start_time("Overall_executing", true);
}

void GraphPlanner::AttemptStatusCallBack(const std_msgs::Bool &msg)
{
    if (command_is_free_nav_) { // current goal is not attemptable
        if (msg.data) {
            ROS_WARN("GP: switch to attemptable planning mode.");
            command_is_free_nav_ = false;
        }
    } else { // current attemptable planning
        if (!msg.data) {
            ROS_WARN("GP: planning without attempting.");
            command_is_free_nav_ = true; 
        }
    }
}

void GraphPlanner::ReEvaluateGoalStatus(const NavNodePtr& goal_ptr, const NodePtrStack& graphIn) {
    if (is_use_internav_goal_) return; // return if using an exsiting internav node as goal
    bool is_position_change =  false;
    // reproject to nearby free space
    int min_layer_dist = DPUtil::layerIdx2Height_.size();
    int min_layer_id = goal_ptr->layer_id;
    for (const auto& node_ptr : graphIn) {
        if (node_ptr == goal_node_ptr_) continue;
        const int cur_layer_diff = origin_layer_id_ - node_ptr->layer_id;
        int adjust_layer_id = node_ptr->layer_id; 
        if (cur_layer_diff > 0) {
            adjust_layer_id = node_ptr->layer_id + std::min(abs(cur_layer_diff), DPUtil::kNeighborLayers);
        } else if (cur_layer_diff < 0) {
            adjust_layer_id = node_ptr->layer_id - std::min(abs(cur_layer_diff), DPUtil::kNeighborLayers);
        }
        const int cur_layer_dist = abs(adjust_layer_id - origin_layer_id_);
        if (cur_layer_dist < min_layer_dist) {
            min_layer_dist = cur_layer_dist;
            min_layer_id = adjust_layer_id;
        }
    }
    // if (min_layer_id != goal_ptr->layer_id) {
    //     goal_ptr->layer_id = min_layer_id;
    //     goal_ptr->position.z = DPUtil::layerIdx2Height_[min_layer_id];
    //     ROS_WARN("GP: goal layer adjusted to %d, with absolute height: %f", min_layer_id, goal_ptr->position.z);
    //     is_position_change = true;
    // }
    if (is_position_change) {
        for (const auto& node_ptr : graphIn) {
            node_ptr->is_block_to_goal = false;
        }
        DynamicGraph::ClearNodeConnectInGraph(goal_node_ptr_);
    }
}

void GraphPlanner::InsertNodes(const NavNodePtr& node_ptr1,
                                const NavNodePtr& node_ptr2) {
    const int M = global_graph_.size();
    NodePtrStack insert_nodes_tmp;
    const PointPair guide_line(node_ptr1->position, node_ptr2->position);

    for (int i = 0; i < M; i++) {
        if (!global_graph_[i]->is_top_layer || global_graph_[i]->is_goal || global_graph_[i]->is_odom) continue;

        for (auto node_ptr : global_graph_[i]->contour_connects) {
            if (std::abs(node_ptr->position.z - global_graph_[i]->position.z) > SameLayerTolerZ) continue;
            // ROS_WARN("Node insert checklist1111111111: %f", (node_ptr->position - node_ptr1->position).norm());

            cv::Point2f node_pos2d(node_ptr->position.x, node_ptr->position.y);
            cv::Point2f top_node2d(global_graph_[i]->position.x, global_graph_[i]->position.y);
            const ConnectPair check_edge(top_node2d, node_pos2d);

            if (ContourGraph::IsEdgeCollideSegment(guide_line, check_edge)) {
                cv::Point2f insert_node2d = ContourGraph::getIntersectionPoint(guide_line, check_edge);
                const Point3D insert_node3d(insert_node2d.x, insert_node2d.y, global_graph_[i]->position.z);
                // check if insert node already exist
                bool is_insert_node_exist = false;
                for (auto inode_ptr : insert_nav_nodes) {
                    if ((insert_node3d - inode_ptr->position).norm() < 0.5) {
                        // cout << "insert node already exist: " << insert_node3d << endl;
                        is_insert_node_exist = true;
                        break;
                    }
                }
                if (is_insert_node_exist) continue;

                NavNodePtr insert_node_ptr = NULL;
                DynamicGraph::CreateNavNodeFromPoint(insert_node3d, global_graph_[i]->layer_id, insert_node_ptr, false, true);

                // TODO: parents should be checked.
                // Put connected_nodes of all inserted_wall_nodes between parents as visible nodes of the inserted_node
                const NavNodePair insert_node_parents(global_graph_[i], node_ptr);
                SetInsertNode(insert_node_ptr, insert_node_parents);
                insert_nav_nodes.push_back(insert_node_ptr);
            }
        }
    }
    for (auto node_ptr : insert_nav_nodes) {
        // // connect between contour nodes and insert node
        // // if (node_ptr->contour_connects.size() == 0) continue;
        // // cout<<"node_ptr->contour_connects.size():"<<node_ptr->contour_connects.size()<<endl;

        // reconnect between insert nodes
        const NodePtrStack cp_insert_nav_nodes = insert_nav_nodes;
        for (auto inode : insert_nav_nodes) {
            if (inode == node_ptr) continue;
            // if (inode->contour_connects.size() == 0) continue;

            if (DPUtil::IsTypeInStack(inode->insert_node_parents.first, node_ptr->connect_nodes) && 
                DPUtil::IsTypeInStack(inode->insert_node_parents.second, node_ptr->connect_nodes)) {
                DynamicGraph::AddEdge(inode, node_ptr);
            }
            if (inode->is_top_layer && node_ptr->is_top_layer) {
                if (ContourGraph::IsTopLayerNodesConnectFreePolygon(inode, node_ptr, false)) {
                    DynamicGraph::AddEdge(inode, node_ptr);
                }
            }
        }
        node_ptr->contour_connects.clear();
        global_graph_.push_back(node_ptr);
        // insert_nav_nodes.push_back(node_ptr);
        // DynamicGraph::AddNodeToGraph(node_ptr);
    }
    insert_nodes_tmp.clear();
}

void GraphPlanner::SetInsertNode(const NavNodePtr& insert_node_ptr, const NavNodePair& insert_node_parents) {
    insert_node_ptr->is_inserted = true;
    insert_node_ptr->is_top_layer = true;
    insert_node_ptr->free_direct = NodeFreeDirect::PILLAR;
    insert_node_ptr->insert_node_parents = insert_node_parents;
    insert_node_ptr->layer_id = insert_node_parents.first->layer_id;

    // Check connectivity with odom and goal node
    if (this->IsValidConnectToOdom(insert_node_ptr, odom_node_ptr_)) {
        DynamicGraph::AddEdge(insert_node_ptr, odom_node_ptr_);
    }    
    if (this->IsValidConnectToGoal(insert_node_ptr, goal_node_ptr_)) {
        DynamicGraph::AddEdge(insert_node_ptr, goal_node_ptr_);
    }

// Connect based on parents
    // connect between graph nodes and insert node
    for (auto cnode : insert_node_parents.first->connect_nodes) {
        if (DPUtil::IsTypeInStack(cnode, insert_node_parents.second->connect_nodes)) {
            DynamicGraph::AddEdge(insert_node_ptr, cnode);
        }
    }

    if (DPUtil::IsTypeInStack(goal_node_ptr_, insert_node_parents.first->connect_nodes) &&
     DPUtil::IsTypeInStack(goal_node_ptr_, insert_node_parents.second->connect_nodes) &&
     DPUtil::IsTypeInStack(insert_node_ptr, goal_node_ptr_->connect_nodes))
        ROS_WARN("GP: goal node connected");

    if (insert_node_parents.first->ctnode != NULL && insert_node_parents.first->ctnode->poly_ptr != NULL) {
        insert_node_ptr->ctnode = insert_node_parents.first->ctnode;
    } else if (insert_node_parents.second->ctnode != NULL && insert_node_parents.second->ctnode->poly_ptr != NULL) {
        insert_node_ptr->ctnode = insert_node_parents.second->ctnode;
    } else {
        insert_node_ptr->ctnode = NULL;
    }
}

void GraphPlanner::ClearInsertNodes(const NodePtrStack& inodes) {
    if (inodes.size() == 0) return;
    int N = inodes.size();
    for (auto i = 0; i < N; i++) {
        auto inode = inodes[i];
        
        NodePtrStack copy_inode_connect = inode->connect_nodes;
        for (auto cnode : copy_inode_connect) {
            DynamicGraph::EraseEdge(inode, cnode);
            DynamicGraph::ClearNavNodeInGraph(inode);
        }
    }

    global_graph_.clear();
}


void GraphPlanner::GetDividedPath(const NodePtrStack& input_path, NodePtrStack& inserted_nodes) {
    inserted_nodes.clear();
    if (input_path.size() <= 2) return;

    int step_size = 0;
    const int N = input_path.size();
    
    while(step_size < N) {
        step_size += 2;

        for (int i = 0; i < N; i+=step_size) {
            this->InsertNodes(input_path[i], input_path[std::min(i + step_size, N-1)]);
        }
        for (int i = 1; i < N; i+=step_size) {
            this->InsertNodes(input_path[i], input_path[std::min(i + step_size, N-1)]);
        }
    }

    UpdateConnectivityBetweenInsertNodes();
    inserted_nodes = insert_nav_nodes;

}

void GraphPlanner::UpdateConnectivityBetweenInsertNodes() {
    if (insert_nav_nodes.size() == 0) return;
    // int N = insert_nav_nodes.size();
    for (auto inode : insert_nav_nodes) {
        for (auto cnode : insert_nav_nodes) {
            if (inode == cnode) continue;
            if (DPUtil::IsTypeInStack(cnode, inode->connect_nodes)) continue;
            if (ContourGraph::IsTopLayerNodesConnectFreePolygon(inode, cnode, false)) {
                    DynamicGraph::AddEdge(inode, cnode);
                }
        }
        if (!inode->is_inserted)
        ROS_ERROR("Insert node not inserted");
    }
}

/****************************** UNUSE CODE ******************************************/

// void GraphPlanner::IterativePathSearch(NodePtrStack& graph,
//                                        const NavNodePtr& odom_node_ptr,
//                                        const NavNodePtr& goal_node_ptr)  {
//     const int M = graph.size();
//     const PointPair guide_line(odom_node_ptr->position, goal_node_ptr->position);
//     // cout<<"GP: M: "<<M<<endl;
//     DPUtil::Timer.start_time("Find_Insert_Nodes");
//     for (int i = 0; i < M; i++) {
//         if (!graph[i]->is_top_layer || graph[i]->is_goal || graph[i]->is_odom) continue;

//         for (auto node_ptr : graph[i]->contour_connects) {
//             if (std::abs(node_ptr->position.z - graph[i]->position.z) > SameLayerTolerZ) continue;
//             // ROS_WARN("Node insert checklist1111111111: %f", (node_ptr->position - odom_node_ptr->position).norm());

//             cv::Point2f node_pos2d(node_ptr->position.x, node_ptr->position.y);
//             cv::Point2f top_node2d(graph[i]->position.x, graph[i]->position.y);
//             const ConnectPair check_edge(top_node2d, node_pos2d);

//             if (ContourGraph::IsEdgeCollideSegment(guide_line, check_edge)) {
//                 cv::Point2f insert_node2d = ContourGraph::getIntersectionPoint(guide_line, check_edge);
//                 const Point3D insert_node3d(insert_node2d.x, insert_node2d.y, graph[i]->position.z);
//                 // check if insert node already exist
//                 bool is_insert_node_exist = false;
//                 for (auto inode_ptr : insert_nav_nodes) {
//                     if ((insert_node3d - inode_ptr->position).norm() < 0.5) {
//                         // cout << "insert node already exist: " << insert_node3d << endl;
//                         is_insert_node_exist = true;
//                         break;
//                     }
//                 }
//                 if (is_insert_node_exist) continue;

//                 NavNodePtr insert_node_ptr = NULL;
//                 DynamicGraph::CreateNavNodeFromPoint(insert_node3d, graph[i]->layer_id, insert_node_ptr, false, true);

//                 // TODO: parents should be checked.
//                 // Put connected_nodes of all inserted_wall_nodes between parents as visible nodes of the inserted_node
//                 const NavNodePair insert_node_parents(graph[i], node_ptr);
//                 SetInsertNode(insert_node_ptr, insert_node_parents);
//                 insert_nav_nodes.push_back(insert_node_ptr);
//             }
//         }
//     }
//     DPUtil::Timer.end_time("Find_Insert_Nodes");
//     DPUtil::Timer.start_time("Insert_Nodes_Connectivity_Check");
//     for (auto node_ptr : insert_nav_nodes) {
//         // connect between contour nodes and insert node
//         if (node_ptr->contour_connects.size() == 0) continue;
//         // cout<<"node_ptr->contour_connects.size():"<<node_ptr->contour_connects.size()<<endl;

//         // reconnect between insert nodes
//         const NodePtrStack cp_insert_nav_nodes = insert_nav_nodes;
//         for (auto inode : cp_insert_nav_nodes) {
//             if (inode == node_ptr) continue;
//             if (inode->contour_connects.size() == 0) continue;

//             if (DPUtil::IsTypeInStack(inode->insert_node_parents.first, node_ptr->connect_nodes) && 
//                 DPUtil::IsTypeInStack(inode->insert_node_parents.second, node_ptr->connect_nodes)) {
//                 // ROS_ERROR("GP: insert node connected");
//                 DynamicGraph::AddEdge(inode, node_ptr);
//             }
//             if (inode->is_top_layer && node_ptr->is_top_layer) {
//                 if (ContourGraph::IsTopLayerNodesConnectFreePolygon(inode, node_ptr, true)) {
//                     DynamicGraph::AddEdge(inode, node_ptr);
//                 }
//             }
//         }
//         node_ptr->contour_connects.clear();
//         graph.push_back(node_ptr);
//         // DynamicGraph::AddNodeToGraph(node_ptr);
//     }
//     DPUtil::Timer.end_time("Insert_Nodes_Connectivity_Check");
// }