#ifndef GRAPH_PLANNER_H
#define GRAPH_PLANNER_H

#include "utility.h"
#include "dynamic_graph.h"
#include "contour_graph.h"

enum ReachVote {
    BLOCK = 0,
    REACH = 1
};

struct GraphPlannerParams {
    GraphPlannerParams() = default;
    float converge_dist;
    float adjust_radius;
    float free_box_dim;
    int   free_thred;
    int   votes_size;
    float goal_z_tolerance;
    int   momentum_thred;
    float momentum_dist;
    int clear_inflate_size;
    bool is_autoswitch;
};


class GraphPlanner {
private:
ros::NodeHandle nh_;
ros::Subscriber attemptable_sub_;

ros::Publisher planning_time_pub_, traverse_time_pub_, reach_goal_pub_;
std_msgs::Bool reached_goal_msg_;

GraphPlannerParams gp_params_;
NavNodePtr goal_node_ptr_ = NULL;
Point3D origin_goal_pos_ = Point3D(0,0,0);
int origin_layer_id_ = -1;
bool is_origin_free_ = false;
bool is_use_internav_goal_ = false;
bool command_is_free_nav_ = false;

NavNodePtr odom_node_ptr_ = NULL;
bool is_goal_init_;
NodePtrStack current_graph_;
bool is_free_nav_goal_;
// momentum planning values
PointStack recorded_path_;
Point3D next_waypoint_;
int path_momentum_counter_;
bool is_global_path_init_;
float last_waypoint_dist_;
Point3D last_planning_odom_;
DPVisualizer gp_viz_;
NodePtrStack global_graph_;

float VERTICAL_ANGLE_COS;

float SameLayerTolerZ = DPUtil::kTolerZ / (DPUtil::kNeighborLayers + 0.5) / 2.0;

float PriorityScore(const NavNodePtr& node_ptr);

bool ReconstructPath(const NavNodePtr& goal_node_ptr,
                     const bool& is_free_nav,
                     PointStack& global_path,
                     NodePtrStack& global_path_ptr);

bool IsNodeConnectInFree(const NavNodePtr& current_node,
                         const NavNodePtr& neighbor_node);

void ReEvaluateGoalStatus(const NavNodePtr& goal_ptr, const NodePtrStack& graphIn);

bool IsValidConnectToGoal(const NavNodePtr& node_ptr, 
                          const NavNodePtr& goal_node_ptr);
                
bool IsValidConnectToOdom(const NavNodePtr& node_ptr, 
                          const NavNodePtr& odom_node_ptr);

Point3D NextNavWaypointFromPath(const PointStack& global_path);

void AttemptStatusCallBack(const std_msgs::Bool& msg);

/* define inline functions */
inline void InitNodesStates(const NodePtrStack& graph) {
    for (const auto& node_ptr : graph) {
        node_ptr->gscore = DPUtil::kINF;
        node_ptr->fgscore = DPUtil::kINF;
        node_ptr->is_reach_goal = false;
        node_ptr->is_traversable = false;
        node_ptr->is_free_traversable = false;
        node_ptr->parent = NULL;
        node_ptr->free_parent = NULL;
    }
}

inline void RecordPathInfo(const PointStack& global_path) {
    if (global_path.size() < 2) {
        ROS_ERROR("GP: recording path for momontum fails, planning path is empty");
        return;
    }
    recorded_path_ = global_path;
    next_waypoint_ = global_path[1];
    last_waypoint_dist_ = (odom_node_ptr_->position - next_waypoint_).norm();
    is_global_path_init_ = true;
    path_momentum_counter_ = 0;
    last_planning_odom_ = global_path[0];
}

inline bool IsResetBlockStatus(const NavNodePtr node_ptr) {
    if (node_ptr->is_odom) return true;
    if (node_ptr->is_near_nodes && DPUtil::IsTypeInStack(node_ptr, odom_node_ptr_->connect_nodes)) {
        return true;
    }
    return false;
}

inline float EulerCost(const NavNodePtr& current_node,
                       const NavNodePtr& neighbor_node) 
{
    return (current_node->position - neighbor_node->position).norm();
}

inline void GoalReset() {
    origin_goal_pos_ = Point3D(0,0,0);
    is_origin_free_ = false;
    origin_layer_id_ = -1;
    command_is_free_nav_ = false;
    if (goal_node_ptr_ != NULL && !is_use_internav_goal_) {
        DynamicGraph::ClearNavNodeInGraph(goal_node_ptr_);
    }
    goal_node_ptr_ = NULL;
}

inline bool IsPointsInSameLevel(const Point3D& p, const Point3D& ref_p) {
    if (abs(p.z - ref_p.z) < gp_params_.goal_z_tolerance) {
        return true;
    }
    return false;
}

void SetInsertNode(const NavNodePtr& insert_node_ptr, const NavNodePair& insert_node_parents);

public:

GraphPlanner() = default;
~GraphPlanner() = default;

void Init(const ros::NodeHandle& nh, const GraphPlannerParams& params);

/**
 * Update Global Graph Traversability Status and gscores
 * @param graph current graph
 * @param odom_node_ptr current odom node ptr
*/
void UpdateGraphTraverability(const NavNodePtr& odom_node_ptr);

/**
 * Path planner on dynamic graph
 * @param global_path(return) return the global path from odom node position
 * @param _nav_goal(return) current navigation waypoint
 * @param _goal_p(return) goal position after free space adjust 
 * @param _is_fails(return) whether the planner fails to find the path
 * @param _is_free_nav(return) the attemptable navigation status (True)->Non-attempts
 * @return whether or not planning success -> publish a valid path for navigation
*/

bool NextGoalPlanning(PointStack& global_path,
                      Point3D& _nav_goal,
                      Point3D& _goal_p,
                      bool& _is_fails,
                      bool& _is_free_nav,
                      NodePtrStack& global_path_ptr);

/**
 * @brief Update connectivity between goal node and navigation graph
 * @param goal_node_ptr new create goal node pointer
 * @param current_graph current navigation graph
 */

void UpdateGoalNavNodeConnects(const NavNodePtr& goal_node_ptr);

/**
 * Graph planner goal update API
 * @param goal goal position
 * @param layer_id the layer id of goal position 
 * @param is_free_nav whether or not the navigation is limited under covered nodes
*/ 
void UpdateGoal(const Point3D& goal, const int& layer_id, const bool& is_free_nav);

/**
 * @brief Reset internal values and containers
 */
inline void ResetPlannerInternalValues() {
    goal_node_ptr_ = NULL; 
    odom_node_ptr_ = NULL;
    
    is_goal_init_         = false;
    is_origin_free_       = false;
    is_use_internav_goal_ = false;
    command_is_free_nav_  = false;
    is_global_path_init_  = false;
    is_origin_free_       = false;
    is_use_internav_goal_ = false;
    command_is_free_nav_  = false;
    is_free_nav_goal_     = false;
    
    current_graph_.clear(); 
    recorded_path_.clear();
    path_momentum_counter_ = 0;
    last_waypoint_dist_    = 0.0;
    origin_goal_pos_    = Point3D(0,0,0);
    next_waypoint_      = Point3D(0,0,0);
    last_planning_odom_ = Point3D(0,0,0);
}

const NodePtrStack& GetNavGraph() const { return global_graph_;};

const NavNodePtr& GetGoalNodePtr() const { return goal_node_ptr_;};
void UpdateGlobalGraph(const NodePtrStack& graph) {global_graph_ = graph;};

NodePtrStack insert_nav_nodes;
bool is_divided_path = false;

void IterativePathSearch(NodePtrStack& graph,
                        const NavNodePtr& odom_node_ptr,
                        const NavNodePtr& goal_node_ptr);

void InsertNodes(const NavNodePtr& node_ptr1,
                const NavNodePtr& node_ptr2);

void GetDividedPath(const NodePtrStack& input_path, NodePtrStack& inserted_nodes);

void ClearInsertNodes(const NodePtrStack& insert_nav_nodes);

void UpdateConnectivityBetweenInsertNodes();

void recheckPath(const NavNodePtr& odom_node_ptr, PointStack& global_path, NodePtrStack& global_path_ptr);

};

#endif