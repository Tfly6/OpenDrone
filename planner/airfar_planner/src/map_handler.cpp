/*
 * Dynamic Route Planner
 * Copyright (C) 2021 Fan Yang - All rights reserved
 * fanyang2@andrew.cmu.edu,   
 */



#include "airfar_planner/map_handler.h"

/***************************************************************************************/

void MapHandler::Init(const MapHandlerParams& params) {
    map_params_ = params;
    row_num_ = std::ceil(map_params_.grid_max_length / map_params_.ceil_length);
    if (row_num_ % 2 == 0) row_num_ ++; // force to odd number, init robot position will be at center
    col_num_ = row_num_;
    level_num_ = std::ceil(map_params_.grid_max_height / map_params_.ceil_height);
    neighbor_Lnum_ = std::ceil(map_params_.sensor_range * 2.0 / map_params_.ceil_length) + 2; 
    neighbor_Hnum_ = map_params_.neighbor_layers * 4 + 3;
    if (level_num_ % 2 == 0) level_num_ ++; // force to odd number, robot will be at center in height
    if (neighbor_Lnum_ % 2 == 0) neighbor_Lnum_ ++; // force to odd number
    // inlitialize grid 
    Eigen::Vector3i pointcloud_grid_size(row_num_, col_num_, level_num_);
    Eigen::Vector3d pointcloud_grid_origin(0,0,0);
    Eigen::Vector3d pointcloud_grid_resolution(map_params_.ceil_length, map_params_.ceil_length, map_params_.ceil_height);
    PointCloudPtr cloud_ptr_tmp;
    world_obs_cloud_grid_ = std::make_unique<grid_ns::Grid<PointCloudPtr>>(
        pointcloud_grid_size, cloud_ptr_tmp, pointcloud_grid_origin, pointcloud_grid_resolution, 3);

    world_free_cloud_grid_ = std::make_unique<grid_ns::Grid<PointCloudPtr>>(
        pointcloud_grid_size, cloud_ptr_tmp, pointcloud_grid_origin, pointcloud_grid_resolution, 3);

    const int N_CELL  = world_obs_cloud_grid_->GetCellNumber();

    for (int i = 0; i < N_CELL; i++) {
        world_obs_cloud_grid_->GetCell(i) = PointCloudPtr(new PointCloud);
        world_free_cloud_grid_->GetCell(i) = PointCloudPtr(new PointCloud);
    }

    global_visited_induces_.resize(N_CELL), util_remove_check_list_.resize(N_CELL);
    util_obs_modified_list_.resize(N_CELL), util_free_modified_list_.resize(N_CELL);
    std::fill(global_visited_induces_.begin(), global_visited_induces_.end(), 0);
    std::fill(util_obs_modified_list_.begin(), util_obs_modified_list_.end(), 0);
    std::fill(util_free_modified_list_.begin(), util_free_modified_list_.end(), 0);
    std::fill(util_remove_check_list_.begin(), util_remove_check_list_.end(), 0);
}

void MapHandler::ResetGripMapCloud() {
    const int N_CELL = world_obs_cloud_grid_->GetCellNumber();
    for (int i=0; i<N_CELL; i++) {
        world_obs_cloud_grid_->GetCell(i)->clear();
        world_free_cloud_grid_->GetCell(i)->clear();
    }
    std::fill(global_visited_induces_.begin(), global_visited_induces_.end(), 0);
    std::fill(util_obs_modified_list_.begin(), util_obs_modified_list_.end(), 0);
    std::fill(util_free_modified_list_.begin(), util_free_modified_list_.end(), 0);
    std::fill(util_remove_check_list_.begin(), util_remove_check_list_.end(), 0);
}

void MapHandler::SetMapOrigin(const Point3D& ori_robot_pos) {
    map_origin_.x = ori_robot_pos.x - (map_params_.ceil_length * row_num_) / 2.0;
    map_origin_.y = ori_robot_pos.y - (map_params_.ceil_length * col_num_) / 2.0;
    map_origin_.z = ori_robot_pos.z - (map_params_.ceil_height * level_num_) / 2.0; // From Ground Level
    Eigen::Vector3d pointcloud_grid_origin(map_origin_.x, map_origin_.y, map_origin_.z);
    world_obs_cloud_grid_->SetOrigin(pointcloud_grid_origin);
    world_free_cloud_grid_->SetOrigin(pointcloud_grid_origin);
    // Reset layer id to height vector
    for (int i=0; i<DPUtil::layerIdx2Height_.size(); i++) {
        Eigen::Vector3i sub(0,0,i);
        Eigen::Vector3d pos = world_obs_cloud_grid_->Sub2Pos(sub);
        DPUtil::layerIdx2Height_[i] = pos.z();
    }
    is_init_ = true;
    ROS_WARN("MH: Map grid initialized.");
}

void MapHandler::UpdateRobotPosition(const Point3D& odom_pos, std::vector<int>& layer_idxs) {
    odom_pos_ = odom_pos;
    if (!is_init_) this->SetMapOrigin(odom_pos_);

    Eigen::Vector3i robot_cell_sub = world_obs_cloud_grid_->Pos2Sub(Eigen::Vector3d(odom_pos_.x, odom_pos_.y, odom_pos_.z));
    // Get neighbor indices
    neighbor_indices_.clear();
    int N = neighbor_Lnum_ / 2;
    int H = neighbor_Hnum_ / 2;
    for (int i = -N; i <= N; i++) {
        for (int j = -N; j <= N; j++) {
            for (int k = -H; k <= H; k++) {
                Eigen::Vector3i neighbor_sub;
                neighbor_sub.x() = robot_cell_sub.x() + i;
                neighbor_sub.y() = robot_cell_sub.y() + j;
                neighbor_sub.z() = robot_cell_sub.z() + k;
                if (world_obs_cloud_grid_->InRange(neighbor_sub)) {
                    int ind = world_obs_cloud_grid_->Sub2Ind(neighbor_sub);
                    neighbor_indices_.push_back(ind);
                }
            }
        }
    }
    layer_idxs.clear();
    for (int k = -H+1; k <= H-1; k++) {
        layer_idxs.push_back(robot_cell_sub.z() + k);
    }
}

void MapHandler::GetSurroundObsCloud(const PointCloudPtr& obsCloudOut, const int& layer_id) {
    if (!is_init_) return;
    obsCloudOut->clear();
    for (const auto& neighbor_ind : neighbor_indices_) {
        Eigen::Vector3i sub = world_obs_cloud_grid_->Ind2Sub(neighbor_ind);
        if (layer_id == -1 || sub.z() == layer_id || sub.z() == layer_id+1 || sub.z() == layer_id-1) {
            if (world_obs_cloud_grid_->GetCell(neighbor_ind)->empty()) continue;
            *obsCloudOut += *(world_obs_cloud_grid_->GetCell(neighbor_ind));
        }
    }
}

void MapHandler::GetSurroundFreeCloud(const PointCloudPtr& freeCloudOut) {
    if (!is_init_) return;
    freeCloudOut->clear();
    for (const auto& neighbor_ind : neighbor_indices_) {
        if (world_free_cloud_grid_->GetCell(neighbor_ind)->empty()) continue;
        *freeCloudOut += *(world_free_cloud_grid_->GetCell(neighbor_ind));
    }
}

void MapHandler::UpdateObsCloudGrid(const PointCloudPtr& obsCloudIn) {
    if (!is_init_ || obsCloudIn->empty()) return;
    std::fill(util_obs_modified_list_.begin(), util_obs_modified_list_.end(), 0);
    for (const auto& point : obsCloudIn->points) {
        Eigen::Vector3i sub = world_obs_cloud_grid_->Pos2Sub(Eigen::Vector3d(point.x, point.y, point.z));
        if (!world_obs_cloud_grid_->InRange(sub)) continue;
        const int ind = world_obs_cloud_grid_->Sub2Ind(sub);
        world_obs_cloud_grid_->GetCell(ind)->points.push_back(point);
        util_obs_modified_list_[ind] = 1;
        global_visited_induces_[ind] = 1;
    }
    // Filter Modified Ceils
    for (int i = 0; i < world_obs_cloud_grid_->GetCellNumber(); ++i) {
      if (util_obs_modified_list_[i] == 1) DPUtil::FilterCloud(world_obs_cloud_grid_->GetCell(i), DPUtil::kLeafSize);
    }
}

void MapHandler::UpdateFreeCloudGrid(const PointCloudPtr& freeCloudIn){
    if (!is_init_ || freeCloudIn->empty()) return;
    std::fill(util_free_modified_list_.begin(), util_free_modified_list_.end(), 0);
    for (const auto& point : freeCloudIn->points) {
        Eigen::Vector3i sub = world_free_cloud_grid_->Pos2Sub(Eigen::Vector3d(point.x, point.y, point.z));
        if (!world_free_cloud_grid_->InRange(sub)) continue;
        const int ind = world_free_cloud_grid_->Sub2Ind(sub);
        world_free_cloud_grid_->GetCell(ind)->points.push_back(point);
        util_free_modified_list_[ind] = 1;
        global_visited_induces_[ind] = 1;
    }
    // Filter Modified Ceils
    for (int i = 0; i < world_obs_cloud_grid_->GetCellNumber(); ++i) {
      if (util_free_modified_list_[i] == 1) DPUtil::FilterCloud(world_free_cloud_grid_->GetCell(i), DPUtil::kLeafSize);
    }
}

void MapHandler::GetNeighborCeilsCenters(PointStack& neighbor_centers) {
    if (!is_init_) return;
    neighbor_centers.clear();
    const int N = neighbor_indices_.size();
    for (int i=0; i<N; i++) {
        const int ind = neighbor_indices_[i];
        if (global_visited_induces_[ind] == 0) continue;
        Point3D center_p(world_obs_cloud_grid_->Ind2Pos(ind));
        neighbor_centers.push_back(center_p);
    }
}

void MapHandler::GetOccupancyCeilsCenters(PointStack& occupancy_centers) {
    if (!is_init_) return;
    occupancy_centers.clear();
    const int N = world_obs_cloud_grid_->GetCellNumber();
    for (int ind=0; ind<N; ind++) {
        if (global_visited_induces_[ind] == 0) continue;
        Point3D center_p(world_obs_cloud_grid_->Ind2Pos(ind));
        occupancy_centers.push_back(center_p);
    }
}

void MapHandler::RemoveObsCloudFromGrid(const PointCloudPtr& obsCloud) {
    std::fill(util_remove_check_list_.begin(), util_remove_check_list_.end(), 0);
    for (const auto& point : obsCloud->points) {
        Eigen::Vector3i sub = world_obs_cloud_grid_->Pos2Sub(Eigen::Vector3d(point.x, point.y, point.z));
        if (!world_free_cloud_grid_->InRange(sub)) continue;
        const int ind = world_free_cloud_grid_->Sub2Ind(sub);
        util_remove_check_list_[ind] = 1;
    }
    for (const auto& ind : neighbor_indices_) {
        if (util_remove_check_list_[ind] == 1 && global_visited_induces_[ind] == 1) {
            DPUtil::RemoveOverlapCloud(world_obs_cloud_grid_->GetCell(ind), obsCloud);
        }
    }
}

