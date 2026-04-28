#ifndef GRAPH_MSGER_FAR_H
#define GRAPH_MSGER_FAR_H

#include "utilityFAR.h"
#include "dynamic_graph.h"

struct GraphMsgerFARParams {
    GraphMsgerFARParams() = default;
    std::string frame_id;
    int   robot_id;
    int   votes_size;
    int   pool_size;
    float dist_margin;
};


class GraphMsgerFAR {
public:
    GraphMsgerFAR() = default;
    ~GraphMsgerFAR() = default;

    void Init(const ros::NodeHandle& nh, const GraphMsgerFARParams& params);

    void UpdateGlobalGraph(const NodePtrStack& graph);

private:
    ros::NodeHandle nh_;
    GraphMsgerFARParams gm_params_;
    ros::Publisher  graph_pub_;
    ros::Subscriber graph_sub_;

    NodePtrStack   global_graph_;
    PointCloudPtr  nodes_cloud_ptr_;
    PointKdTreePtr kdtree_graph_cloud_;
    
    void CreateDecodedNavNode(const airfar_planner::VisNode& vnode, NavNodePtr& node_ptr);

    inline bool IsEncodeType(const NavNodePtr& node_ptr) {
        if (node_ptr->is_odom || !node_ptr->is_finalized || FARUtil::IsOutsideGoal(node_ptr)) {
            return false;
        }
        return true;
    }

    inline bool IsMismatchFreeNode(const NavNodePtr& nearest_ptr, const airfar_planner::VisNode& vnode) {
        if (nearest_ptr == NULL) return false;
        const bool is_navpoint = vnode.is_navpoint == 0 ? false : true;
        const bool is_boundary = vnode.is_boundary == 0 ? false : true;
        if (nearest_ptr->is_navpoint != is_navpoint || nearest_ptr->is_boundary != is_boundary) return true;
        return false;
    }

    inline NavNodePtr IdToNodePtr(const std::size_t& node_id, const IdxMap& idx_map, const NodePtrStack& node_stack) {
        const auto it = idx_map.find(node_id);
        if (it != idx_map.end()) {
            const std::size_t idx = it->second;
            if (idx < node_stack.size()) {
                return node_stack[idx];
            }
        }
        return NULL;
    }

    inline airfar_planner::VisGraph nav2vis(const airfar_planner::NavGraphConstPtr& graphIn) {
        airfar_planner::VisGraph graphOut;
        graphOut.header = graphIn->header;
        graphOut.robot_id = graphIn->robot_id;
        graphOut.nodes.resize(graphIn->nodes.size());
        for (std::size_t i = 0; i < graphIn->nodes.size(); ++i) {
            graphOut.nodes[i].header = graphIn->nodes[i].header;
            graphOut.nodes[i].id = graphIn->nodes[i].id;
            graphOut.nodes[i].FreeType = graphIn->nodes[i].FreeType;
            graphOut.nodes[i].position = graphIn->nodes[i].position;
            graphOut.nodes[i].surface_dirs = graphIn->nodes[i].surface_dirs;
            graphOut.nodes[i].is_covered = graphIn->nodes[i].is_covered;
            graphOut.nodes[i].is_frontier = graphIn->nodes[i].is_frontier;
            graphOut.nodes[i].is_navpoint = graphIn->nodes[i].is_navpoint;
            graphOut.nodes[i].is_boundary = graphIn->nodes[i].is_boundary;
            graphOut.nodes[i].connect_nodes = graphIn->nodes[i].connect_nodes;
            graphOut.nodes[i].poly_connects = graphIn->nodes[i].poly_connects;
            graphOut.nodes[i].contour_connects = graphIn->nodes[i].contour_connects;
            graphOut.nodes[i].trajectory_connects = graphIn->nodes[i].trajectory_connects;
        }
        graphOut.size = graphIn->size;
        return graphOut;
    }

    NavNodePtr NearestNodePtrOnGraph(Point3D p, float radius);

    void EncodeGraph(const NodePtrStack& graphIn, airfar_planner::VisGraph& graphOut);

    void GraphCallBack(const airfar_planner::NavGraphConstPtr& msg);

    void PublishGlobalGraph(const NodePtrStack& graphIn);

    bool PublishGraphService(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

    void ExtractConnectIdxs(const airfar_planner::VisNode& node,
                            IdxStack& connect_idxs,
                            IdxStack& poly_idxs,
                            IdxStack& contour_idxs,
                            IdxStack& traj_idxs);

};


#endif