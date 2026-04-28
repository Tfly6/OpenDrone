#ifndef CONTOUR_GRAPH_H
#define CONTOUR_GRAPH_H

#include "utility.h"

struct ConnectPair
{
    cv::Point2f start_p;
    cv::Point2f end_p;

    ConnectPair() = default;
    ConnectPair(cv::Point2f p1, cv::Point2f p2):start_p(p1), end_p(p2) {}
    ConnectPair(Point3D p1, Point3D p2) {
        this->start_p.x = p1.x;
        this->start_p.y = p1.y;
        this->end_p.x = p2.x;
        this->end_p.y = p2.y;
    }
    
    bool operator ==(const ConnectPair& pt) const 
    {
        return (this->start_p == pt.start_p && this->end_p == pt.end_p) || (this->start_p == pt.end_p && this->end_p == pt.start_p);
    }
};

struct ContourGraphParams {
    ContourGraphParams() = default;
    float kAroundDist;
    float kPillarPerimeter;
    int KD_TREE_K;
    int knn_search_num_;
    float knn_search_radius_;
    float wall_insert_factor;
    float voxel_dim;
};

class ContourGraph {
public:
    ContourGraph() = default;
    ~ContourGraph() = default;

    void Init(const ContourGraphParams& params);

    static std::vector<CTNodeStack>  multi_contour_graph_;
    static std::vector<PolygonStack> multi_contour_polygons_;
    static std::vector<std::vector<PointPair>> multi_global_contour_;
    std::vector<std::unique_ptr<cv::flann::Index>> multi_contour_graph_kdtree_;
    bool is_kdtree_built_ = false;


    ContourGraphParams ctgraph_params_;
    float ALIGN_ANGLE_COS;
    std::unique_ptr<cv::flann::KDTreeIndexParams> indexParams;

    NavNodePtr odom_node_ptr_ = NULL;
    
    // static functions
    void UpdateContourGraph(const NavNodePtr& odom_node_ptr,
                            const int& layer_idx,
                            const std::vector<std::vector<Point3D>>& filtered_contours);

    /* Match current contour with global navigation nodes */
    void MatchContourWithNavGraph(const NodePtrStack& nav_graph,
                                  const std::vector<int>& cur_layer_idxs,
                                  CTNodeStack& new_convex_vertices);

    void ExtractGlobalContours(const NodePtrStack& nav_graph, const std::vector<int>& cur_layers);

    // Connect Vertical Edges based on KNN search
    void ConnectVerticalEdges(const int& layer_id);

    static bool IsNavNodesConnectFromContour(const NavNodePtr& node_ptr1, 
                                          const NavNodePtr& node_ptr2);

    static bool IsCTNodesConnectFromContour(const CTNodePtr& ctnode1, 
                                            const CTNodePtr& ctnode2);

    static bool IsNavNodesConnectFreePolygon(const NavNodePtr& node_ptr1,
                                             const NavNodePtr& node_ptr2,
                                             const bool& is_local_only=false,
                                             const bool& layer_limited=true);

    static bool IsNavToGoalConnectFreePolygon(const NavNodePtr& node_ptr,
                                              const NavNodePtr& goal_ptr);
    static bool IsNavToOdomConnectFreePolygon(const NavNodePtr& node_ptr,
                                              const NavNodePtr& odom_ptr);                                        

    // static bool IsPoint3DConnectFreePolygon(const Point3D& p1, const Point3D& p2, 
    //                                         const int& start_layer, const int& end_layer);

    static bool IsPointsConnectFreePolygon(const ConnectPair& cedge, 
                                           const int& start_layer,
                                           const int& end_layer,
                                           const bool& is_global_check,
                                           const bool& layer_limited=true);

    static bool IsPointsConnectFreePolygonTest(const ConnectPair& cedge,
                                                const int& start_layer,
                                                const int& end_layer,
                                                const bool& is_global_check,
                                                const bool& layer_limited);

    static bool IsTopLayerNodesConnectFreePolygon(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2, const bool& layer_limited);

    static bool IsEdgeCollideSegment(const PointPair& line, const ConnectPair& edge);

    static cv::Point2f getIntersectionPoint(const PointPair& line, const ConnectPair& edge);
    // static bool ReprojectPointOutsidePolygons(Point3D& point, const float& free_radius);

    bool IsPointInVetexAngleRestriction(const CTNodePtr& ctnode, const Point3D end_p);

private:
    /* static private functions */
    inline void AddCTNodeToGraph(const CTNodePtr& ctnode_ptr, const int& layer_id) {
        if (ctnode_ptr == NULL && ctnode_ptr->free_direct == NodeFreeDirect::UNKNOW) {
            ROS_ERROR_THROTTLE(1.0, "CG: Add ctnode to contour graph fails, ctnode is invaild.");
            return;
        }
        ContourGraph::multi_contour_graph_[layer_id].push_back(ctnode_ptr);
    }

    inline void AddPolyToContourPolygon(const PolygonPtr& poly_ptr, const int& layer_id) {
        if (poly_ptr == NULL || poly_ptr->vertices.empty()) {
            ROS_ERROR_THROTTLE(1.0, "CG: Add polygon fails, polygon is invaild.");
            return;
        }
        ContourGraph::multi_contour_polygons_[layer_id].push_back(poly_ptr);
    } 

    inline void AddConnect(const CTNodePtr& ctnode_ptr1, const CTNodePtr& ctnode_ptr2) {
        if (ctnode_ptr1 != ctnode_ptr2 &&
            !DPUtil::IsTypeInStack(ctnode_ptr2, ctnode_ptr1->connect_nodes) &&
            !DPUtil::IsTypeInStack(ctnode_ptr1, ctnode_ptr2->connect_nodes))
        {
            ctnode_ptr1->connect_nodes.push_back(ctnode_ptr2);
            ctnode_ptr2->connect_nodes.push_back(ctnode_ptr1);
        }
    }

    template <typename NodeType>
    static inline cv::Point2f NodeProjectDir(const NodeType& node) {
        cv::Point2f project_dir(0,0);
        if (node->free_direct != NodeFreeDirect::PILLAR && node->free_direct != NodeFreeDirect::UNKNOW) {
            const Point3D topo_dir = DPUtil::SurfTopoDirect(node->surf_dirs);
            if (node->free_direct == NodeFreeDirect::CONCAVE) {
                project_dir = cv::Point2f(topo_dir.x, topo_dir.y);
            } else {
                project_dir = cv::Point2f(-topo_dir.x, -topo_dir.y);
            }
        }
        return project_dir;
    }

    static inline ConnectPair ReprojectEdge(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2, const float& dist, const bool& is_inv=false);

    static void UpdateOdomFreePosition(const NavNodePtr& odom_ptr, Point3D& global_free_p);

    static bool IsEdgeCollidePoly(const PointStack& poly, const ConnectPair& edge);

    inline static bool IsNeedGlobalCheck(const Point3D& p1, const Point3D& p2, const Point3D& robot_p) {
        if ((p1 - robot_p).norm() > DPUtil::kSensorRange || (p2 - robot_p).norm() > DPUtil::kSensorRange) {
            return true;
        }
        return false;
    }

    inline void ClearContourGraph(const int& layer_id) {
        ContourGraph::multi_contour_graph_[layer_id].clear();
        ContourGraph::multi_contour_polygons_[layer_id].clear(); 
    }

    bool IsAPillarPolygon(const PointStack& vertex_points);

    void CreateCTNode(const Point3D& pos, const int& layer_id, CTNodePtr& ctnode_ptr, const PolygonPtr& poly_ptr, const bool& is_pillar);

    void CreatePolygon(const PointStack& poly_points, const int& layer_id, PolygonPtr& poly_ptr);

    NavNodePtr NearestNavNodeForCTNode(const CTNodePtr& ctnode_ptr, const NodePtrStack& nav_graph, const int& layer_id);

    void AnalysisConvexityOfCTNode(const CTNodePtr& ctnode_ptr);
    
    /* Analysis CTNode surface angle */
    void AnalysisSurfAngleAndConvexity(const CTNodeStack& contour_graph);

    // calculate KD-Tree on contour graph
    void BuildKDTreeOnContourGraph(const CTNodeStack& contour_graph, const int& layer_id);
    void SearchKNN(const CTNodePtr& node, std::vector<int>& indices, const int& layer_id);
    bool InsertWallNodes(CTNodePtr& start, CTNodePtr& end, const int& layer_idx);
    void SetWallCornerNodes(const CTNodePtr& node_ptr1, const CTNodePtr& node_ptr2, const int& layer_idx);

};

#endif