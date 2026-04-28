#ifndef TERRIAN_PLANNER_H
#define TERRIAN_PLANNER_H

#include "utility.h"
#include "planner_visualizer.h"

struct TerrainPlannerParams {
    TerrainPlannerParams() = default;
    std::string world_frame;
    float local_range;
    float radius;
    float voxel_size;
    int inflate_size;
};

struct TerrianNode {
    TerrianNode() = default;
    int id;
    Point3D position;
    bool is_occupied;
    float fscore;
    float gscore;
    std::shared_ptr<TerrianNode> parent;
};

typedef std::shared_ptr<TerrianNode> TerrianNodePtr;
typedef std::vector<TerrianNode> TerrianNodeStack;


struct TNodeptr_fcomp
{
  bool operator()(const TerrianNodePtr& n1, const TerrianNodePtr& n2) const
  {
    return n1->fscore > n2->fscore;
  }
};

class TerrainPlanner {

public:
    TerrainPlanner() = default;
    ~TerrainPlanner() = default;

    void Init(const ros::NodeHandle& nh, const TerrainPlannerParams& params);

    void UpdateCenterNode(const NavNodePtr& node_ptr);
    
    void SetLocalTerrainObsCloud(const PointCloudPtr& obsCloudIn);
    
    bool PlanPathFromPToP(const Point3D& from_p, const Point3D& to_p, PointStack& path);

    inline bool PlanPathFromNodeToNode(const NavNodePtr& node_from, const NavNodePtr& node_to, PointStack& path) {
        return PlanPathFromPToP(node_from->position, node_to->position, path); 
    }
    
    void VisualPaths();

private:
    ros::NodeHandle nh_;
    TerrainPlannerParams tp_params_;
    int row_num_, col_num_;
    bool is_grids_init_ = false;
    NavNodePtr center_node_prt_ = NULL;
    Point3D center_pos_;
    std::vector<PointStack> viz_path_stack_;

    ros::Publisher local_path_pub_, terrian_map_pub_;

    std::unique_ptr<grid_ns::Grid<TerrianNodePtr>> terrain_grids_;

    void ExtractPath(const TerrianNodePtr& end_ptr, PointStack& path);

    void GridVisualCloud();

    inline float EulerCost(const TerrianNodePtr& node_ptr1, const TerrianNodePtr& node_ptr2) {
        return (node_ptr1->position - node_ptr2->position).norm();
    }

    inline void AllocateGridNodes() {
        std::cout<<"size of terrain nodes: " <<terrain_grids_->GetCellNumber()<<std::endl;
        for (std::size_t i=0; i<terrain_grids_->GetCellNumber(); i++) {
            terrain_grids_->GetCell(i) = std::make_shared<TerrianNode>();
            terrain_grids_->GetCell(i)->id = i;
        }
    }

    inline void ResetGridsOccupancy() {
        if (!is_grids_init_) return;
        for (std::size_t i=0; i<terrain_grids_->GetCellNumber(); i++) {
            terrain_grids_->GetCell(i)->is_occupied = false;
        }
    }

    inline void ResetGridsPositions() {
        if (!is_grids_init_) return;
        for (std::size_t i=0; i<terrain_grids_->GetCellNumber(); i++) {
            terrain_grids_->GetCell(i)->position = Point3D(terrain_grids_->Ind2Pos(i));
        }
    }

    inline void ResetGridsPlanStatus() {
        if (!is_grids_init_) return;
        for (std::size_t i=0; i<terrain_grids_->GetCellNumber(); i++) {
            terrain_grids_->GetCell(i)->fscore = DPUtil::kINF;
            terrain_grids_->GetCell(i)->gscore = DPUtil::kINF;
            terrain_grids_->GetCell(i)->parent = NULL;
        }
    }

    inline Point3D Ind2Point3D(const int& ind) {
        return terrain_grids_->GetCell(ind)->position;
    }

    inline PCLPoint Ind2PCLPoint(const int& ind) {
        return DPUtil::Point3DToPCLPoint(Ind2Point3D(ind));
    }

};


#endif