#ifndef DYNAMIC_GRAPH_H
#define DYNAMIC_GRAPH_H

#include "utility.h"
#include "contour_graph.h"
#include "terrain_planner.h"


struct DynamicGraphParams {
    DynamicGraphParams() = default;
    float near_dist;
    float sensor_range;
    float move_thred;
    int dumper_thred;
    int finalize_thred;
    int pool_size;
    int votes_size;
    int terrian_inflate;
    float traj_interval_ratio;
    float kConnectAngleThred;
    float filter_pos_margin;
    float filter_dirs_margin;
    float wall_insert_factor;
};

class DynamicGraph {  
private:
    Point3D robot_pos_;
    NavNodePtr odom_node_ptr_ = NULL;
    NavNodePtr cur_internav_ptr_ = NULL;
    NavNodePtr last_internav_ptr_ = NULL;
    NodePtrStack new_nodes_;
    static std::size_t id_tracker_; // Odom node owns default ID "1", Goal Node owns default ID "0"
    std::vector<int> cur_layer_idxs_;
    std::vector<int> wide_layer_idxs_;
    DynamicGraphParams dg_params_;
    NodePtrStack near_nav_nodes_, wide_near_nodes_, internav_near_nodes_, surround_internav_nodes_;
    float CONNECT_ANGLE_COS, NOISE_ANGLE_COS, ALIGN_ANGLE_COS, TRAJ_DIST;
    static float VERTICAL_ANGLE_COS;
    bool is_bridge_internav_ = false;
    Point3D last_connect_pos_;

    static NodePtrStack globalGraphNodes_;

    TerrainPlanner terrain_planner_;
    TerrainPlannerParams tp_params_;

    bool NodeLocalPerception(const NavNodePtr& node_ptr,
                             bool& _is_wall_end,
                             const bool& is_nearby_update = true);

    bool IsInDirectConstraint(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2);

    bool IsInContourDirConstraint(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2);

    bool IsInterNavpointNecessary();
    
    void ReEvaluateConvexity(const NavNodePtr& node_ptr);

    bool IsOldNodesAround(const CTNodePtr& ctnode, const float& radius);
    bool IsOldWallNodesAround(const CTNodePtr& ctnode, const float& radius);

    /* Merge two nodes in Graph into one remain node, (mark one node as merged)*/
    void MergeNodeInGraph(const NavNodePtr& node_ptr1, 
                          const NavNodePtr& node_ptr2);

    /* check whether there is a connection in similar diection */
    bool IsSimilarConnectInDiection(const NavNodePtr& node_ptr_from,
                                    const NavNodePtr& node_ptr_to);

    bool IsAShorterConnectInDir(const NavNodePtr& node_ptr_from, const NavNodePtr& node_ptr_to);

    bool UpdateNodePosition(const NavNodePtr& node_ptr, const Point3D& new_pos);

    static void InitNodePosition(const NavNodePtr& node_ptr, const Point3D& new_pos); 

    bool UpdateNodeSurfDirs(const NavNodePtr& node_ptr, PointPair cur_dirs);

    void ReOrganizeGraphConnect();

    bool ReEvaluateCorner(const NavNodePtr node_ptr);

    void RecordContourEdge(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2);

    void DeleteContourEdge(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2);

    void UpdateGlobalNearNodes();

    bool IsNodeFullyCovered(const NavNodePtr& node_ptr);

    bool ReEvaluateConnectUsingTerrian(const NavNodePtr& node_ptr1, const NavNodePtr node_ptr2);



    /* Assign ID to new navigation node */
    static inline void AssignGlobalNodeID(const NavNodePtr& node_ptr) {
        if (node_ptr->is_goal) {
            node_ptr->id = 0;
            return;
        }
        if (node_ptr->is_odom) {
            node_ptr->id = 1;
            return;
        }
        node_ptr->id = id_tracker_;
        id_tracker_ ++;
    }

    inline void UpdateCurInterNavNode(const NavNodePtr& internav_node_ptr) {
        if (internav_node_ptr == NULL || !internav_node_ptr->is_navpoint) return;
        cur_internav_ptr_ = internav_node_ptr;
        if (last_internav_ptr_ == NULL) { // init inter navigation nodes
            terrain_planner_.UpdateCenterNode(cur_internav_ptr_);
            last_internav_ptr_ = cur_internav_ptr_;
        } else if (last_internav_ptr_ != cur_internav_ptr_) {
            terrain_planner_.UpdateCenterNode(cur_internav_ptr_);
            if (DPUtil::IsTrajectory) {
                this->AddTrajectoryConnect(cur_internav_ptr_, last_internav_ptr_);
            }
            last_internav_ptr_ = cur_internav_ptr_;
        } 
    }

    inline void AddTrajectoryConnect(const NavNodePtr& cur_node, const NavNodePtr& last_node) {
        if (last_node == NULL || cur_node == NULL) return;
        if (!DPUtil::IsTypeInStack(last_node, cur_node->trajectory_connects) &&
            !DPUtil::IsTypeInStack(cur_node, last_node->trajectory_connects)) 
        {
            std::deque<int> vote_q1(dg_params_.votes_size, 1);
            std::deque<int> vote_q2(dg_params_.votes_size, 1);
            cur_node->trajectory_votes.insert({last_node->id, vote_q1});
            last_node->trajectory_votes.insert({cur_node->id, vote_q1});
            cur_node->trajectory_connects.push_back(last_node);
            last_node->trajectory_connects.push_back(cur_node);
        }
    }

    inline void RecordValidTrajEdge(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
        const auto it1 = node_ptr1->trajectory_votes.find(node_ptr2->id);
        const auto it2 = node_ptr2->trajectory_votes.find(node_ptr1->id);
        if (it1 == node_ptr1->trajectory_votes.end() || it2 == node_ptr2->trajectory_votes.end() || it1->second.size() != it2->second.size()) {
            ROS_ERROR("Trajectory vote map error.");
            return;
        }
        it1->second.push_back(1), it2->second.push_back(1);
        if (it1->second.size() > dg_params_.votes_size) {
            it1->second.pop_front(), it2->second.pop_front();
        }
    }

    inline void ConvertWideLayerIdxs(const std::vector<int>& cur_layer_idxs, std::vector<int>& wide_layer_idxs) {
        wide_layer_idxs = cur_layer_idxs;
        if (cur_layer_idxs.empty()) return;
        if (cur_layer_idxs.front() > 0) {
            wide_layer_idxs.insert(wide_layer_idxs.begin(), cur_layer_idxs.front()-1);
        }
        if (cur_layer_idxs.back() < DPUtil::layerIdx2Height_.size()-1) {
            wide_layer_idxs.push_back(cur_layer_idxs.back()+1);
        }
    }

    inline void RemoveInValidTrajEdge(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
        const auto it1 = node_ptr1->trajectory_votes.find(node_ptr2->id);
        const auto it2 = node_ptr2->trajectory_votes.find(node_ptr1->id);
        if (it1 == node_ptr1->trajectory_votes.end() || it2 == node_ptr2->trajectory_votes.end() || it1->second.size() != it2->second.size()) {
            ROS_ERROR("Trajectory vote map error.");
            return;
        }
        it1->second.push_back(0), it2->second.push_back(0);
        if (it1->second.size() > dg_params_.votes_size) {
            it1->second.pop_front(), it2->second.pop_front();
        }
        if (!DPUtil::IsVoteTrue(it1->second)) {
            ROS_WARN("DG: trajectory edge disconnected, no traversable path found.");
            // clear potential connections
            DPUtil::EraseNodeFromStack(node_ptr2, node_ptr1->trajectory_connects);
            DPUtil::EraseNodeFromStack(node_ptr1, node_ptr2->trajectory_connects);

        }
    }

    /* Define inline functions */
    inline bool SetNodeToClear(const NavNodePtr& node_ptr, const bool& is_direct_delete=false) {
        if (DPUtil::IsFreeNavNode(node_ptr)) return false;
        if (is_direct_delete) {
            node_ptr->is_merged = true;
            return true;
        }else {
            node_ptr->clear_dumper_count ++;
            if (node_ptr->clear_dumper_count > dg_params_.dumper_thred) {
                node_ptr->is_merged = true;
                return true;
            }
        }
        return false;
    }

    inline void ReduceDumperCounter(const NavNodePtr& node_ptr) {
        if (DPUtil::IsFreeNavNode(node_ptr)) return;
        if (node_ptr->clear_dumper_count > 0) {
            node_ptr->clear_dumper_count --;
        }
    }

    inline bool IsInternavInRange(const NavNodePtr& cur_inter_ptr) {
        if (cur_inter_ptr == NULL) return false;
        const float dist_thred = TRAJ_DIST;
        const int layer_dist = abs(cur_inter_ptr->layer_id - odom_node_ptr_->layer_id);
        if (cur_inter_ptr->fgscore > dist_thred || layer_dist >= DPUtil::kNeighborLayers) {
            return false;
        }
        return true;
    }

    static inline bool IsPolygonEdgeVoteTrue(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
        const auto it1 = node_ptr1->edge_votes.find(node_ptr2->id);
        const auto it2 = node_ptr2->edge_votes.find(node_ptr1->id);
        if (it1 != node_ptr1->edge_votes.end() && it2 != node_ptr2->edge_votes.end()) {
            if (DPUtil::IsVoteTrue(it1->second)) {
                // DEBUG
                if (!DPUtil::IsVoteTrue(it2->second)) ROS_ERROR_THROTTLE(1.0, "DG: Polygon edge vote result are not matched.");
                return true;
            }
        }
        return false;
    }

    inline void RedirectContourConnect(const NavNodePtr& node_ptr) {
        if (node_ptr->contour_connects.empty()) return;
        const int N = node_ptr->contour_votes.size();
        for (std::size_t i=0; i<N; i++) {
            if (node_ptr->contour_connects[i]->is_merged) continue; 
            for (std::size_t j=0; j<N; j++) {
                if (i==j || j>i || node_ptr->contour_connects[j]->is_merged) continue;
                const Point3D dir1 = (node_ptr->contour_connects[i]->position - node_ptr->position).normalize();
                const Point3D dir2 = (node_ptr->contour_connects[j]->position - node_ptr->position).normalize(); 
                if (dir1 * dir2 < 0.0) {
                    this->RecordContourEdge(node_ptr->contour_connects[i], node_ptr->contour_connects[j]);
                }
            }
        }
    }

    inline bool IsConnectedNewNode(const NavNodePtr& node_ptr) {

        if (DPUtil::IsFreeNavNode(node_ptr) || IsConnectedNode(node_ptr)) return true; 
        return false;
    }

    inline bool IsConnectedNode(const NavNodePtr& node_ptr) {
        if ((node_ptr->is_contour_match && node_ptr->ctnode->poly_ptr->is_connect) ||
            DPUtil::IsTypeInStack(node_ptr, odom_node_ptr_->connect_nodes)) 
        {
            return true;
        }
        return false;
    }

    inline bool IsVisibleNode(const NavNodePtr& node_ptr) {
        if ((node_ptr->is_contour_match && node_ptr->ctnode->poly_ptr->is_visiable) || 
            DPUtil::IsTypeInStack(node_ptr, odom_node_ptr_->connect_nodes))
        {
            return true;
        }
        return false;
    }

    /* Create new navigation node, and return a shared pointer to it */
    inline void CreateNewNavNodeFromContour(const CTNodePtr& ctnode_ptr, NavNodePtr& node_ptr) {
        CreateNavNodeFromPoint(ctnode_ptr->position, ctnode_ptr->layer_id, node_ptr, false);
        node_ptr->is_contour_match = true;
        node_ptr->ctnode = ctnode_ptr;
        node_ptr->free_direct = ctnode_ptr->free_direct;
        node_ptr->is_wall_corner = ctnode_ptr->is_wall_corner;
        node_ptr->is_wall_insert = ctnode_ptr->is_wall_insert;
        UpdateNodeSurfDirs(node_ptr, ctnode_ptr->surf_dirs);
    }

    static inline void ClearContourConnectionInGraph(const NavNodePtr& node_ptr) {
        // reset connected contour pairs if exists
        if (!node_ptr->potential_contours.empty()) {
            const std::size_t N = node_ptr->potential_contours.size();
            for (std::size_t i=0; i<N; i++) {
                for (std::size_t j=0; j<N; j++) {
                    if (i == j || j > i) continue;
                    const NavNodePtr cnode1 = node_ptr->potential_contours[i];
                    const NavNodePtr cnode2 = node_ptr->potential_contours[j];
                    const auto it1 = cnode1->contour_votes.find(cnode2->id);
                    if (it1 != cnode1->contour_votes.end()) {
                        // reset cnode1's cnode2
                        it1->second.clear(), it1->second.push_back(0);
                        // reset cnode2's cnode1
                        const auto it2 = cnode2->contour_votes.find(cnode1->id);
                        it2->second.clear(), it2->second.push_back(0);
                    }
                }
            }
        }
        // this->RedirectContourConnect(node_ptr);
        for (const auto& ct_cnode_ptr : node_ptr->contour_connects) {
            DPUtil::EraseNodeFromStack(node_ptr, ct_cnode_ptr->contour_connects);
        }
        for (const auto& pt_cnode_ptr : node_ptr->potential_contours) {
            DPUtil::EraseNodeFromStack(node_ptr, pt_cnode_ptr->potential_contours);
            pt_cnode_ptr->contour_votes.erase(node_ptr->id);
        }
        node_ptr->contour_connects.clear();
        node_ptr->contour_votes.clear();
        node_ptr->potential_contours.clear();
    }

    static inline bool IsMergedNode(const NavNodePtr& node_ptr) {
        if (DPUtil::IsFreeNavNode(node_ptr)) return false;
        if (node_ptr->is_merged) {
            return true;
        }
        return false;
    }

    inline void ResetNodeFilters(const NavNodePtr& node_ptr) {
        node_ptr->is_finalized = false;
        node_ptr->pos_filter_vec.clear();
        node_ptr->surf_dirs_vec.clear();
    }

    inline void ResetNodeConnectVotes(const NavNodePtr& node_ptr) {
        // reset contours
        for (const auto& pcnode : node_ptr->potential_contours) {
            const auto it1 = node_ptr->contour_votes.find(pcnode->id);
            const auto it2 = pcnode->contour_votes.find(node_ptr->id);
            if (DPUtil::IsVoteTrue(it1->second)) {
                it1->second.clear(), it1->second.push_back(1);
                it2->second.clear(), it2->second.push_back(1);
            } else {
                it1->second.clear(), it1->second.push_back(0);
                it2->second.clear(), it2->second.push_back(0);
            }
        }
        // reset polygon connections
        for (const auto& pcnode : node_ptr->potential_edges) {
            const auto it1 = node_ptr->edge_votes.find(pcnode->id);
            const auto it2 = pcnode->edge_votes.find(node_ptr->id);
            if (DPUtil::IsVoteTrue(it1->second)) {
                it1->second.clear(), it1->second.push_back(1);
                it2->second.clear(), it2->second.push_back(1);
            } else {
                it1->second.clear(), it1->second.push_back(0);
                it2->second.clear(), it2->second.push_back(0);
            }
        }
    }

    /* Clear nodes in global graph which is marked as merge */
    inline void ClearMergedNodesInGraph() {
        for (const auto& node_ptr : globalGraphNodes_) {
            if (IsMergedNode(node_ptr)) {
                ClearNodeConnectInGraph(node_ptr);
                ClearContourConnectionInGraph(node_ptr);
            }
        }
        const auto new_end = std::remove_if(globalGraphNodes_.begin(), globalGraphNodes_.end(), IsMergedNode);
        globalGraphNodes_.resize(new_end - globalGraphNodes_.begin());
        UpdateGlobalNearNodes();
    }

public:
    DynamicGraph() = default;
    ~DynamicGraph() = default;

    void Init(const ros::NodeHandle& nh, const DynamicGraphParams& params);

    /**
     *  Updtae robot pos and odom node 
     *  @param robot_pos current robot position in world frame
     *  @param is_robot_stop(return) whether or not robot is stop
    */
    bool UpdateOdom(const Point3D& robot_pos, const std::vector<int>& cur_layers, bool& is_robot_stop);
    
    /**
     * Extract Navigation Nodes from Vertices Detected -> Update [new_nodes_] internally.
     * @param new_ctnodes new contour vertices without matching global navigation node
     * @return return false if there was no new navigation node extracted, otherwise, return true.
    */
    bool ExtractGraphNodes(const CTNodeStack& new_ctnodes);

    /**
     * Update Entire Navigation Graph with given new navigation nodes --
     * 1. clear false positive nodes detection
     * 2. Update Edges between exsiting nodes
     * 3. Adding edges between existing nodes with new extracted nodes
     * @param new_nodes new extracted navigation nodes given for graph updating
     * @param clear_nodes existing nodes which is now recoginzed as false positive
    */
    void UpdateNavGraph(const NodePtrStack& new_nodes,
                        NodePtrStack& clear_node);

    /* Static Functions */

    static void DeletePolygonEdge(const NavNodePtr& node_ptr1, 
                                  const NavNodePtr& node_ptr2, 
                                  const int& queue_size,
                                  const bool& is_reset=false);

    static void RecordPolygonEdge(const NavNodePtr& node_ptr1, 
                                  const NavNodePtr& node_ptr2, 
                                  const int& queue_size,
                                  const bool& is_reset=false);

    static inline void CreateNavNodeFromPoint(const Point3D& point, const int& layer_id, NavNodePtr& node_ptr, const bool& is_odom, 
                                              const bool& is_navpoint=false, const bool& is_goal=false) 
    {
        node_ptr = std::make_shared<NavNode>();
        AssignGlobalNodeID(node_ptr);
        node_ptr->layer_id = layer_id;
        node_ptr->pos_filter_vec.clear();
        node_ptr->surf_dirs_vec.clear();
        node_ptr->ctnode = NULL;
        node_ptr->is_contour_match = false;
        node_ptr->is_odom = is_odom;
        node_ptr->is_intermediate = false;  
        node_ptr->is_near_nodes = true;
        node_ptr->is_wide_near = true;
        node_ptr->is_merged = false;
        node_ptr->is_frontier = (is_odom || is_navpoint || is_goal) ? false : true;
        node_ptr->is_finalized = is_navpoint ? true : false;
        node_ptr->is_traversable = is_odom;
        node_ptr->is_navpoint = is_navpoint;
        node_ptr->is_goal = is_goal;
        node_ptr->clear_dumper_count = 0;
        node_ptr->finalize_counter = 0;
        node_ptr->connect_nodes.clear();
        node_ptr->contour_connects.clear();
        node_ptr->contour_votes.clear();
        node_ptr->potential_contours.clear();
        node_ptr->trajectory_connects.clear();
        node_ptr->trajectory_votes.clear();
        node_ptr->node_type = NodeType::NOT_DEFINED; // TODO: define different node type
        node_ptr->free_direct = (is_odom || is_navpoint) ? NodeFreeDirect::PILLAR : NodeFreeDirect::UNKNOW;
        InitNodePosition(node_ptr, point);
        node_ptr->near_odom_dist = DPUtil::kINF;
    }

    static inline void ClearNodeConnectInGraph(const NavNodePtr& node_ptr) {
        // clear navigation connections
        for (const auto& cnode_ptr: node_ptr->connect_nodes) {
            DPUtil::EraseNodeFromStack(node_ptr, cnode_ptr->connect_nodes);
        }
        for (const auto& pt_cnode_ptr : node_ptr->potential_edges) {
            DPUtil::EraseNodeFromStack(node_ptr, pt_cnode_ptr->potential_edges);
            pt_cnode_ptr->edge_votes.erase(node_ptr->id);
        }
        node_ptr->connect_nodes.clear();
        node_ptr->edge_votes.clear();
        node_ptr->potential_edges.clear();
    }

    static inline bool IsConnectInVerticalConstrain(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
        // if (DPUtil::IsFreeNavNode(node_ptr1) || DPUtil::IsFreeNavNode(node_ptr2)) return true;
        // check maximum vertical angle
        if (node_ptr1->layer_id != node_ptr2->layer_id) {
            const float v_angle = fabs((node_ptr2->position-node_ptr1->position).norm_dot(Point3D(0,0,1)));
            if (v_angle > VERTICAL_ANGLE_COS) {
                return false;
            }
        }
        return true;
    }

    static inline void ClearNavNodeInGraph(const NavNodePtr& node_ptr) {
        ClearNodeConnectInGraph(node_ptr);
        ClearContourConnectionInGraph(node_ptr);
        DPUtil::EraseNodeFromStack(node_ptr, globalGraphNodes_);
    }

    /* Add new navigation node to global graph */
    static inline void AddNodeToGraph(const NavNodePtr& node_ptr) {
        if (node_ptr != NULL) {
            globalGraphNodes_.push_back(node_ptr);
        } else {
            ROS_WARN("DG: exist new node pointer is NULL, fails to add into graph");
        }
    }

    /* Add edge for given two navigation nodes */
    static inline void AddEdge(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
        if (node_ptr1 == node_ptr2) return;
        if (!DPUtil::IsTypeInStack(node_ptr2, node_ptr1->connect_nodes) &&
            !DPUtil::IsTypeInStack(node_ptr1, node_ptr2->connect_nodes)) 
        {
            node_ptr1->connect_nodes.push_back(node_ptr2);
            node_ptr2->connect_nodes.push_back(node_ptr1);
        }
    }

    /* Erase connection between given two nodes */
    static inline void EraseEdge(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
        // clear node2 in node1's connection
        DPUtil::EraseNodeFromStack(node_ptr2, node_ptr1->connect_nodes);
        // clear node1 in node2's connection 
        DPUtil::EraseNodeFromStack(node_ptr1, node_ptr2->connect_nodes);
    }

    /* Clear Current Graph */
    inline void ResetCurrentGraph() {
        odom_node_ptr_     = NULL; 
        cur_internav_ptr_  = NULL; 
        last_internav_ptr_ = NULL;

        id_tracker_         = 2;
        is_bridge_internav_ = false;
        last_connect_pos_   = Point3D(0,0,0);
        
        near_nav_nodes_.clear(); 
        wide_near_nodes_.clear(); 
        internav_near_nodes_.clear();
        surround_internav_nodes_.clear();
        new_nodes_.clear();
        globalGraphNodes_.clear();
    }

    /* Get Internal Values */
    const NavNodePtr GetOdomNode() const { return odom_node_ptr_;};
    const NodePtrStack& GetNavGraph() const { return globalGraphNodes_;};
    const NodePtrStack& GetNearNavGraph() const { return near_nav_nodes_;};
    const NodePtrStack& GetWideNavGraph() const { return wide_near_nodes_;};
    const NodePtrStack& GetNewNodes() const { return new_nodes_;};
    const NavNodePtr& GetLastInterNavNode() const { return last_internav_ptr_;};

    void ConnectVerticalNodes(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2);

    void AddVerticalEdges(const NavNodePtr& node_ptr, const NavNodePtr& connected_ptr);
    void AddUpwardEdges(const NavNodePtr& node_ptr, const NavNodePtr& connected_ptr);
    void AddDownwardEdges(const NavNodePtr& node_ptr, const NavNodePtr& connected_ptr);

    /* Evaluate exist edges */
    bool IsValidConnect(const NavNodePtr& node_ptr1, 
                        const NavNodePtr& node_ptr2,
                        const bool& is_local_only,
                        const bool& is_check_contour=true,
                        const bool& is_layer_limited=true);
    
    bool IsValidConnect(const NavNodePtr& node_ptr1, 
                        const NavNodePtr& node_ptr2,
                        const bool& is_local_only,
                        const bool& is_check_contour,
                        bool& _is_merge,
                        bool& _is_matched,
                        const bool& is_layer_limited);

    bool IsValidVertConnect(const NavNodePtr& node_ptr1, 
                        const NavNodePtr& node_ptr2,
                        const bool& is_local_only,
                        const bool& is_check_contour=true,
                        const bool& is_layer_limited=true);
    
    bool IsValidVertConnect(const NavNodePtr& node_ptr1, 
                        const NavNodePtr& node_ptr2,
                        const bool& is_local_only,
                        const bool& is_check_contour,
                        bool& _is_merge,
                        bool& _is_matched,
                        bool& is_layer_limited);
};



#endif

/************************** UNUSE CODE ****************************/

// inline void ResetGraphContourMatchStatus() {
//     for (const auto& node_ptr : globalGraphNodes_) {
//         node_ptr->is_contour_match = false;
//         node_ptr->ctnode = NULL;
//     }
// }