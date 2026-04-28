/*
 * Dynamic Route Planner
 * Copyright (C) 2021 Fan Yang - All rights reserved
 * fanyang2@andrew.cmu.edu,   
 */



#include "airfar_planner/utility.h"

/***************************************************************************************/

void DPUtil::FilterCloud(const PointCloudPtr& point_cloud, const float& leaf_size) {
  // filter point cloud with constant leaf size 0.2m
  pcl::PointCloud<PCLPoint> filter_rs_cloud;
  pcl::VoxelGrid<PCLPoint> vg;
  vg.setInputCloud(point_cloud);
  vg.setLeafSize(leaf_size, leaf_size, leaf_size);
  vg.filter(filter_rs_cloud);
  *point_cloud = filter_rs_cloud;
}

void DPUtil::RemoveNanInfPoints(const PointCloudPtr& cloudInOut) {
  pcl::PointCloud<PCLPoint> temp_cloud;
  temp_cloud.resize(cloudInOut->points.size());
  std::size_t idx = 0;
  for (const auto& p : cloudInOut->points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      ROS_WARN_ONCE("DPUtil: nan or inf point detected.");
      continue;
    }
    // if (!is_graph_init_ && p.z < 0.2) continue;
    if (p.z > -0.2 && p.z < 0.2) continue;
    temp_cloud.points[idx] = p;
    idx ++;
  }
  temp_cloud.points.resize(idx);
  *cloudInOut = temp_cloud;
}

PCLPoint DPUtil::Point3DToPCLPoint(const Point3D& point) {
  PCLPoint pcl_point;
  pcl_point.x = point.x;
  pcl_point.y = point.y;
  pcl_point.z = point.z;
  pcl_point.intensity = point.intensity;
  return pcl_point;
}

void DPUtil::ExtractNewObsPointCloud(const PointCloudPtr& cloudIn,
                                     const PointCloudPtr& cloudRefer,
                                     const PointCloudPtr& cloudNew,
                                     const bool& is_stack_cloud)
{
  PointCloudPtr temp_new_cloud(new pcl::PointCloud<PCLPoint>());
  PointCloudPtr current_new_cloud(new pcl::PointCloud<PCLPoint>());
  DPUtil::ResetCloudIntensity(cloudIn, false);
  DPUtil::ResetCloudIntensity(cloudRefer, true);
  current_new_cloud->clear(), temp_new_cloud->clear();
  *temp_new_cloud = *cloudIn + *cloudRefer;
  DPUtil::FilterCloud(temp_new_cloud, DPUtil::kLeafSize * 2.0);
  for (const auto& p : temp_new_cloud->points) {
    if (p.intensity < DPUtil::kNewPIThred) {
      current_new_cloud->points.push_back(p);
    } 
  }
  if (is_stack_cloud) {
    *cloudNew += *current_new_cloud;
    DPUtil::FilterCloud(cloudNew, DPUtil::kLeafSize);
  } else {
    cloudNew->clear();
    *cloudNew = *current_new_cloud;
  }
}

void DPUtil::ResetCloudIntensity(const PointCloudPtr& cloudIn, const bool isHigh) {
  const float kValue = isHigh? 255.0 : 0.0;
  for (std::size_t i=0; i<cloudIn->size(); i++) {
    cloudIn->points[i].intensity = kValue;
  }
}

void DPUtil::ResetCloudIntensityWithTime(const PointCloudPtr& cloudInOut) {
  const float curTime = ros::Time::now().toSec() - DPUtil::systemStartTime;
  for (std::size_t i=0; i<cloudInOut->size(); i++) {
    cloudInOut->points[i].intensity = curTime;
  }
}


void DPUtil::CropPCLCloud(const PointCloudPtr& cloudIn,
                          const PointCloudPtr& cloudCropped,
                          const Point3D& centriod,
                          const float& range) 
{
  const std::size_t cloud_size = cloudIn->size();
  std::size_t idx = 0;
  cloudCropped->clear(), cloudCropped->resize(cloud_size);
  for (const auto& p : cloudIn->points) {
    if ((centriod - p).norm() < range) {
      cloudCropped->points[idx] = p;
      idx ++;
    }
  }
  cloudCropped->resize(idx);
}

void DPUtil::CropPCLCloud(const PointCloudPtr& cloudInOut,
                          const Point3D& centriod,
                          const float& range) 
{
  PointCloudPtr temp_crop_ptr(new pcl::PointCloud<PCLPoint>());
  DPUtil::CropPCLCloud(cloudInOut, temp_crop_ptr, centriod, range);
  *cloudInOut = *temp_crop_ptr;
}

void DPUtil::CropCloudWithinHeight(const PointCloudPtr& cloudInOut, const float& height) {
  pcl::PointCloud<PCLPoint> temp_cloud;
  temp_cloud.resize(cloudInOut->points.size());
  std::size_t idx = 0;
  for (const auto& p : cloudInOut->points) {
    if (!DPUtil::IsPointInToleratedHeight(p, height)) continue;
    temp_cloud.points[idx] = p;
    idx ++;
  }
  temp_cloud.points.resize(idx);
  *cloudInOut = temp_cloud;
}

void DPUtil::TransformPCLFrame(const std::string& from_frame_id,
                               const std::string& to_frame_id,
                               const tf::TransformListener* tf_listener,
                               const PointCloudPtr& cloudInOut) 
{
  if (cloudInOut->empty()) return;
  pcl::PointCloud<PCLPoint> aft_tf_cloud;
  tf::StampedTransform cloud_to_map_tf;
  try {
    tf_listener->waitForTransform(to_frame_id, from_frame_id, ros::Time(0), ros::Duration(1.0));
    tf_listener->lookupTransform(to_frame_id, from_frame_id, ros::Time(0), cloud_to_map_tf);
  } catch (tf::TransformException ex){
    throw ex;
    return;
  }
  pcl_ros::transformPointCloud(*cloudInOut, aft_tf_cloud, cloud_to_map_tf);
  *cloudInOut = aft_tf_cloud;
}

void DPUtil::TransformPoint3DFrame(const std::string& from_frame_id,
                                   const std::string& to_frame_id,
                                   const tf::TransformListener* tf_listener,
                                   Point3D& point)
{
  tf::Vector3 point_vec(point.x, point.y, point.z);
  tf::StampedTransform transform_tf_stamp;
  try {
    tf_listener->waitForTransform(to_frame_id, from_frame_id, ros::Time(0), ros::Duration(1.0));
    tf_listener->lookupTransform(to_frame_id, from_frame_id, ros::Time(0), transform_tf_stamp);
    point_vec = transform_tf_stamp * point_vec;
  } catch (tf::TransformException ex){
    ROS_ERROR("Tracking Point3D TF lookup: %s",ex.what());
    return;
  }
  point.x = point_vec.x();
  point.y = point_vec.y();
  point.z = point_vec.z();
}

bool DPUtil::IsSameFrameID(const std::string& cur_frame, const std::string& ref_frame) {
  std::string str1 = cur_frame;
  std::string str2 = ref_frame;
  if (cur_frame[0] == '/') str1 = cur_frame.substr(1);
  if (ref_frame[0] == '/') str2 = ref_frame.substr(1);
  return str1 == str2;
}

void DPUtil::ConvertCloudToPCL(const PointStack& point_stack, 
                               const PointCloudPtr& point_cloud_ptr) 
{
  point_cloud_ptr->clear();
  point_cloud_ptr->points.resize(point_stack.size());
  std::size_t i = 0;
  for (const auto& p : point_stack) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.x)) {
      ROS_WARN_ONCE("DPUtil: nan or inf point detected.");
      continue;
    }
    PCLPoint point;
    point.x = p.x;
    point.y = p.y;
    point.z = p.z;
    point.intensity = p.intensity;
    point_cloud_ptr->points[i] = point;
    i++;
  }
}

geometry_msgs::Point DPUtil::Point3DToGeoMsgPoint(const Point3D& point) {
  geometry_msgs::Point p;
  p.x = point.x;
  p.y = point.y;
  p.z = point.z;
  return p;
}

void DPUtil::ExtractFreeAndObsCloud(const PointCloudPtr& newCloudIn,
                                    const PointCloudPtr& freeCloudOut,
                                    const PointCloudPtr& obsCloudOut) 
{
  // pre-process cloud
  freeCloudOut->clear(), obsCloudOut->clear();
  const std::size_t cloud_size = newCloudIn->size();
  freeCloudOut->resize(cloud_size), obsCloudOut->resize(cloud_size);
  std::size_t free_idx = 0;
  std::size_t obs_idx = 0;
  // iteratte through points
  for (const auto& p : newCloudIn->points) {
    if (p.intensity < DPUtil::kFreeZ) {
      freeCloudOut->points[free_idx] = p;
      free_idx ++;
    } else {
      obsCloudOut->points[obs_idx] = p;
      obs_idx ++;
    }
  } 
  freeCloudOut->resize(free_idx), obsCloudOut->resize(obs_idx);
}

void DPUtil::ExtractFreeAndObsFromScanCloud(const PointCloudPtr& scanCloudIn,
                                            const PointCloudPtr& terrainCloudIn,
                                            const PointCloudPtr& freeCloudOut,
                                            const PointCloudPtr& obsCloudOut) 
{
  // pre-process cloud
  freeCloudOut->clear(), obsCloudOut->clear();
  const std::size_t cloud_size = terrainCloudIn->size();
  pcl::copyPointCloud(*scanCloudIn, *obsCloudOut);
  freeCloudOut->resize(cloud_size);
  std::size_t free_idx = 0;
  // iteratte through points
  for (const auto& p : terrainCloudIn->points) {
    if (p.intensity < DPUtil::kFreeZ) {
      freeCloudOut->points[free_idx] = p;
      free_idx ++;
    }
  } 
  freeCloudOut->resize(free_idx);
  DPUtil::RemoveOverlapCloud(obsCloudOut, freeCloudOut);
}

void DPUtil::UpdateKdTrees(const PointCloudPtr& newObsCloudIn,
                           const PointCloudPtr& freeCloudIn,
                           const PointCloudPtr& obsCloudIn) 
{
  if (!newObsCloudIn->empty()) {
    DPUtil::kdtree_new_cloud_->setInputCloud(newObsCloudIn);
  } else {
    DPUtil::ClearKdTree(newObsCloudIn, DPUtil::kdtree_new_cloud_);
  }
}

void DPUtil::ClearKdTree(const PointCloudPtr& cloud_ptr,
                         const PointKdTreePtr& kdTree_ptr) {
  PCLPoint temp_p = DPUtil::Point3DToPCLPoint(DPUtil::robot_pos);
  cloud_ptr->resize(1), cloud_ptr->points[0] = temp_p;
  kdTree_ptr->setInputCloud(cloud_ptr);
}

bool DPUtil::IsPointNearNewPoints(const Point3D& p, const bool& is_creation) {
  std::size_t near_c = DPUtil::PointInNewCounter(p, DPUtil::kNearDist * 2.0);
  const int counter_limit = is_creation ? std::floor(DPUtil::KNewPointC / 2.0) : DPUtil::KNewPointC;
  return (near_c > counter_limit) ? true : false;
}

std::size_t DPUtil::PointInXCounter(const Point3D& p,
                                    const float& radius,
                                    const PointKdTreePtr& KdTree) 
{
  std::vector<int> pointSearchInd;
  std::vector<float> pointSearchSqDis;
  PCLPoint pcl_p;
  pcl_p.x = p.x, pcl_p.y = p.y, pcl_p.z = p.z;
  if (!std::isfinite(pcl_p.x) || !std::isfinite(pcl_p.y) || !std::isfinite(pcl_p.z)) {
    return 0;
  }
  KdTree->radiusSearch(pcl_p, radius, pointSearchInd, pointSearchSqDis);
  return pointSearchInd.size();
}

std::size_t DPUtil::PointInNewCounter(const Point3D& p, const float& radius) {
  return DPUtil::PointInXCounter(p, radius, DPUtil::kdtree_new_cloud_);
}

void DPUtil::Flat3DPointCloud(const PointCloudPtr& cloudIn, 
                              const PointCloudPtr& cloudFlat,
                              const bool& is_downsample) 
{
  cloudFlat->clear();
  pcl::copyPointCloud(*cloudIn, *cloudFlat);
  const std::size_t cloud_size = cloudFlat->size();
  for (std::size_t i=0; i<cloud_size; i++) {
    cloudFlat->points[i].z = 0.0;
  }
  if (is_downsample) {
    DPUtil::FilterCloud(cloudFlat, DPUtil::kLeafSize);
  }
}

int DPUtil::Mod(const int& a, const int& b) {
  return (b + (a % b)) % b;
}

void DPUtil::EraseNodeFromStack(const NavNodePtr& node_ptr,
                                NodePtrStack& node_stack) {
  for (auto it = node_stack.begin(); it != node_stack.end(); it++) {
    if (*it  == node_ptr) {
      node_stack.erase(it--);
    }
  }
}

void DPUtil::EraseNodeFromMap(const NavNodePtr& node_ptr,
                              NavMap& nav_map) {
  for (auto it = nav_map.begin(); it != nav_map.end();) {
    if (it->first  == node_ptr) {
      it = nav_map.erase(it);
    } else {
      ++it;
    }
  }
}

bool DPUtil::IsSamePoint3D(const Point3D& p1, const Point3D& p2) {
  if ((p2 - p1).norm() < DPUtil::kEpsilon) {
    return true;
  }
  return false;
}

void DPUtil::SetDifference(std::vector<int>& v1, std::vector<int>& v2, std::vector<int>& diff) {
  std::sort(v1.begin(), v1.end());
  std::sort(v2.begin(), v2.end());
  diff.clear();
  std::set_difference(v1.begin(), v1.end(), v2.begin(), v2.end(), std::inserter(diff, diff.begin()));
}

void DPUtil::ExtractSurroundCloud(const PointCloudPtr& worldObsCloudIn, const PointCloudPtr& worldFreeCloudIn,
                                  const PointCloudPtr& surroundObsCloudOut, const PointCloudPtr& surroundFreeCloudOut)
{

  DPUtil::CropPCLCloud(worldObsCloudIn, 
                       surroundObsCloudOut, 
                       DPUtil::robot_pos, DPUtil::kSensorRange + DPUtil::kNearDist * 2.5); // add 2.0m to reduce margin effect
  DPUtil::CropPCLCloud(worldFreeCloudIn, 
                       surroundFreeCloudOut, 
                       DPUtil::robot_pos, DPUtil::kSensorRange + DPUtil::kNearDist * 2.5); // add 2.0m to reduce margin effect
}

bool DPUtil::IsInSurfacePairs(const Point3D& diff_p,
                              const PointPair& surf_dirs) 
{
  const Point3D F_diff_p(diff_p.x, diff_p.y, 0);
  if (F_diff_p.norm() < DPUtil::kEpsilon) return true;
  const Point3D norm_dir = F_diff_p.normalize();
  const float dist = F_diff_p.norm();
  float margin_angle_noise = DPUtil::kAngleNoise;
  const float noise_max_dist = 2.0 * DPUtil::kNearDist;
  if (dist * sin(margin_angle_noise) < noise_max_dist) {
    margin_angle_noise = std::asin(noise_max_dist / std::max(dist, noise_max_dist));
  }
  Point3D temp_opposite_dir;
  // check first half range
  temp_opposite_dir = -surf_dirs.second;
  // DEBUG
  if (temp_opposite_dir != (Point3D(0,0,0) - surf_dirs.second)) {
    ROS_ERROR("Point3D negative calcualtion error.");
  }
  float in_res_thred = DPUtil::NoiseCosValue(surf_dirs.first * temp_opposite_dir, true, margin_angle_noise);
  if (norm_dir * surf_dirs.first > in_res_thred &&
      norm_dir * temp_opposite_dir > in_res_thred) {
    return true;
  }
  // check second half range
  temp_opposite_dir = -surf_dirs.first;
  in_res_thred = DPUtil::NoiseCosValue(surf_dirs.second * temp_opposite_dir, true, margin_angle_noise);
  if (norm_dir * surf_dirs.second > in_res_thred &&
      norm_dir * temp_opposite_dir > in_res_thred) {
    return true;
  }
  return false;
}

bool DPUtil::IsInCoverageDirPairs(const Point3D& diff_p,
                                  const NavNodePtr& node_ptr) 
{
  const Point3D f_diff_p(diff_p.x, diff_p.y, 0);
  if (node_ptr->free_direct == NodeFreeDirect::PILLAR || f_diff_p.norm() < DPUtil::kEpsilon) return false;
  const Point3D norm_dir = f_diff_p.normalize();
  const PointPair surf_dirs = node_ptr->surf_dirs;
  const float dot_value = surf_dirs.first * surf_dirs.second;
  if (node_ptr->free_direct == NodeFreeDirect::CONCAVE) {
    if (norm_dir * surf_dirs.first > dot_value &&
        norm_dir * surf_dirs.second > dot_value) {
      return true;
    }
  } else if (node_ptr->free_direct == NodeFreeDirect::CONVEX) {
    if (norm_dir * (-surf_dirs.second) > dot_value &&
      norm_dir * (-surf_dirs.first) > dot_value) {
      return true;
    }
  }
  return false;
}

bool DPUtil::IsInContourDirPairs(const Point3D& diff_p,
                                 const PointPair& surf_dirs) 
{
  const float dist = diff_p.norm();
  float margin_angle_noise = DPUtil::kAngleNoise;
  const float noise_max_dist = 2.0 * DPUtil::kNearDist;
  if (dist * sin(margin_angle_noise) < noise_max_dist) {
    margin_angle_noise = std::asin(noise_max_dist / std::max(dist, noise_max_dist));
  }
  margin_angle_noise = std::min(margin_angle_noise * 2.0, M_PI / 4.0);
  // check first half range
  const float dot_value1 = surf_dirs.first.norm_dot(diff_p);
  if (std::acos(dot_value1) < margin_angle_noise) {
    return true;
  }
  // check second half range
  const float dot_value2 = surf_dirs.second.norm_dot(diff_p);
  if (std::acos(dot_value2) < margin_angle_noise) {
    return true;
  }
  return false;
}

float DPUtil::DirsDistance(const PointPair& ref_dirs, const PointPair& compare_dirs) {
  const float a_d1 = std::acos(ref_dirs.first.norm_dot(compare_dirs.first));
  const float a_d2 = std::acos(ref_dirs.second.norm_dot(compare_dirs.second));
  return a_d1 + a_d2;
}

Point3D DPUtil::SurfTopoDirect(const PointPair& dirs, bool& _is_wall) {
  Point3D topo_dir = dirs.first + dirs.second;
  topo_dir.z = 0.0;
  _is_wall = false;
  if (topo_dir.norm() <= 2*DPUtil::kEpsilon) {
    ROS_WARN_THROTTLE(1.0, "DP: surface topo direction creation fails, mark convexity as unknown.");
    _is_wall = true;
    return Point3D(0,0,0);
  } else {
    return topo_dir.normalize();
  }
}

Point3D DPUtil::SurfTopoDirect(const PointPair& dirs) {
  bool UNUSE_iswall;
  return DPUtil::SurfTopoDirect(dirs, UNUSE_iswall);
}

void DPUtil::InflateCloud(const PointCloudPtr& obsCloudInOut, 
                          const float& resol,
                          const int& inflate_size,
                          const bool& deep_down_inflate) 
{
  const std::size_t current_size = obsCloudInOut->size();
  const int z_size = inflate_size + 1;
  const int z_down_idx = deep_down_inflate ? z_size + 1 : z_size;
  const std::size_t array_size = current_size * (pow(inflate_size*2+1, 2)*(z_size*(z_size+z_down_idx)+1) + 1);
  obsCloudInOut->resize(array_size);
  std::size_t cur_idx = current_size;
  for (std::size_t p_idx=0; p_idx<current_size; p_idx++) {
    PCLPoint p = obsCloudInOut->points[p_idx];
    for (int ix=-inflate_size; ix<=inflate_size; ix++) {
      for (int iy=-inflate_size; iy<=inflate_size; iy++) {
        for (int iz=-z_size-z_down_idx; iz<=z_size; iz++) {
          PCLPoint ref_p;
          ref_p.x = p.x + ix * resol;
          ref_p.y = p.y + iy * resol;
          ref_p.z = p.z + iz * resol;
          ref_p.intensity = p.intensity;
          obsCloudInOut->points[cur_idx] = ref_p;
          cur_idx ++;
        }
      }
    }
  }
  obsCloudInOut->resize(cur_idx);
  DPUtil::FilterCloud(obsCloudInOut, resol);
}

float DPUtil::NoiseCosValue(const float& dot_value, const bool& is_large, const float& noise) {
  const float crop_value = DPUtil::ClampAbsRange(dot_value, 1.0);
  const float theta = std::acos(dot_value);
  const int sign = is_large ? 1 : -1;
  double margin_theta = theta + sign * noise;
  margin_theta = std::min(std::max(margin_theta, 0.0), M_PI);
  return cos(margin_theta);
}

void DPUtil::ClusterFilterCloud(const PointCloudPtr& cloudInOut,
                                const float& radius,
                                const std::size_t c_thred) {
  PointCloudPtr temp_ptr(new pcl::PointCloud<PCLPoint>());
  if (cloudInOut->empty()) return;
  DPUtil::kdtree_filter_cloud_->setInputCloud(cloudInOut);
  temp_ptr->clear(), temp_ptr->resize(cloudInOut->size());
  std::size_t idx = 0;
  for (const auto& p : cloudInOut->points) {
    const std::size_t counter = DPUtil::PointInXCounter(Point3D(p), radius, DPUtil::kdtree_filter_cloud_);
    if (counter > c_thred) {
      temp_ptr->points[idx] = p;
      idx ++;
    }
  }
  temp_ptr->resize(idx), *cloudInOut = *temp_ptr;
  return;
}

void DPUtil::TransferCloud(const Point3D& transPoint, const PointCloudPtr& cloudInOut) {
  PointCloudPtr temp_ptr(new pcl::PointCloud<PCLPoint>());
  pcl::copyPointCloud(*cloudInOut, *temp_ptr);
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  transform(0,3) = transPoint.x;
  transform(1,3) = transPoint.y;
  transform(2,3) = transPoint.z;
  pcl::transformPointCloud(*temp_ptr, *cloudInOut, transform);
}

void DPUtil::ExtractOverlapCloud(const PointCloudPtr& cloudIn,
                                 const PointCloudPtr& cloudRef,
                                 const PointCloudPtr& cloudOverlapOut,
                                 const float& margin_ratio)
{
  PointCloudPtr temp_cloud(new pcl::PointCloud<PCLPoint>());
  DPUtil::ResetCloudIntensity(cloudIn, true);
  DPUtil::ResetCloudIntensity(cloudRef, false);
  *temp_cloud = *cloudIn + *cloudRef;
  const float leaf_size = margin_ratio * DPUtil::kLeafSize;;
  DPUtil::FilterCloud(temp_cloud, leaf_size);
  cloudOverlapOut->clear(), cloudOverlapOut->resize(temp_cloud->size());
  std::size_t idx = 0;
  for (const auto& p : temp_cloud->points) {
    if (p.intensity > 0.0 && p.intensity < 255.0) {
      cloudOverlapOut->points[idx] = p;
      idx ++;
    }
  }
  cloudOverlapOut->resize(idx);
}

void DPUtil::RemoveOverlapCloud(const PointCloudPtr& cloudInOut,
                                const PointCloudPtr& cloudRef,
                                const bool& is_copy_cloud) 
{
  if (cloudRef->empty() || cloudInOut->empty()) return;
  PointCloudPtr temp_cloud(new pcl::PointCloud<PCLPoint>());
  PointCloudPtr ref_cloud = cloudRef;
  if (is_copy_cloud) {
    PointCloudPtr copyRefCloud(new pcl::PointCloud<PCLPoint>());
    pcl::copyPointCloud(*cloudRef, *copyRefCloud);
    ref_cloud = copyRefCloud;
  }
  DPUtil::ResetCloudIntensity(cloudInOut, true);
  DPUtil::ResetCloudIntensity(ref_cloud, false);
  *temp_cloud = *cloudInOut + *ref_cloud;
  const float leaf_size = DPUtil::kLeafSize * 1.25;
  DPUtil::FilterCloud(temp_cloud, leaf_size);
  cloudInOut->clear(), cloudInOut->resize(temp_cloud->size());
  std::size_t idx = 0;
  for (const auto& p : temp_cloud->points) {
    if (p.intensity < 255.0) continue;
    cloudInOut->points[idx] = p;
    idx++;
  }
  cloudInOut->resize(idx);
}

void DPUtil::StackCloudByTime(const PointCloudPtr& curInCloud,
                              const PointCloudPtr& StackCloud,
                              const float& duration) 
{
  pcl::PointIndices::Ptr outliers(new pcl::PointIndices());
  outliers->indices.resize(StackCloud->size());
  DPUtil::ResetCloudIntensityWithTime(curInCloud);
  *StackCloud += *curInCloud;
  std::size_t idx = 0;
  std::size_t outIdx = 0;
  const float curTime = ros::Time::now().toSec() - DPUtil::systemStartTime;
  for (const auto& pcl_p : StackCloud->points) {
    if (curTime - pcl_p.intensity > duration) {
      outliers->indices[outIdx] = idx;
      outIdx ++;
    }
    idx ++;
  }
  outliers->indices.resize(outIdx);
  DPUtil::RemoveIndicesFromCloud(outliers, StackCloud);
}

void DPUtil::RemoveIndicesFromCloud(const pcl::PointIndices::Ptr& outliers,
                                    const PointCloudPtr& cloudInOut) {
  pcl::ExtractIndices<PCLPoint> extract;
  extract.setInputCloud(cloudInOut);
  extract.setIndices(outliers);
  extract.setNegative(true);
  extract.filter(*cloudInOut);
}

float DPUtil::ApproxAtan2(const float& y, const float x) {
  const float ay = std::abs(y);
  const float ax = std::abs(x);
  const float n1 = 0.97239411f;
  const float n2 = -0.19194795f;
  const float z = ay > ax ? ax / ay : ay / ax;  // [0,1]
  float th = (n1 + n2 * z * z) * z;
  if (ay > ax)
    th = M_PI_2 - th;  // [0,π/2]
  if (x < 0)
    th = M_PI - th;      // [0,π]
  th = copysign(th, y);  // [-π,π]
  return th;
}

float DPUtil::PixelDistance(const cv::Point2f& pre_p, const cv::Point2f& cur_p) {
  return std::hypotf(pre_p.x - cur_p.x, pre_p.y - cur_p.y);
}

float DPUtil::VerticalDistToLine(const Point3D& start_p, 
                                 const Point3D& end_p, 
                                 const Point3D& cur_p,
                                 const bool& is_segment_restrict) 
{
  const Point3D line_dir = end_p - start_p;
  const Point3D diff_p  = cur_p - start_p;
  const float dot_value = line_dir.norm_dot(diff_p);
  if (is_segment_restrict) {
    const Point3D ops_line_dir = start_p - end_p;
    const Point3D ops_diff_dir = cur_p - end_p;
    if (dot_value < 0.0 || ops_line_dir.norm_dot(ops_diff_dir) < 0.0) {
      return DPUtil::kINF;
    }
  }
  return sin(acos(dot_value)) * diff_p.norm();
}

Point3D DPUtil::ContourSurfDirs(const Point3D& end_p, 
                                const Point3D& start_p, 
                                const Point3D& center_p,
                                const float& radius) 
{
  const float D = (center_p - end_p).norm();
  const float phi = std::acos((center_p - end_p).norm_dot(start_p - end_p));
  const float H = D * sin(phi);
  const float theta = asin(H / radius);
  const Point3D dir = (start_p - end_p).normalize();
  const Point3D V_p = end_p + dir * D * cos(phi);
  const Point3D K_p = V_p - dir * radius * cos(theta);
  Point3D surf_dir = K_p - center_p;
  surf_dir.z = 0.0, surf_dir = surf_dir.normalize();
  return surf_dir;
}

float DPUtil::CosinTheta(const Point3D& vertex, const Point3D& p1, const Point3D& p2) {
  const Point3D dir1 = p1 - vertex;
  const Point3D dir2 = p2 - vertex;
  return dir1.norm_dot(dir2);
}

void DPUtil::CorrectDirectOrder(const PointPair& ref_dir, PointPair& dirInOUt) {
  PointPair temp_dir = dirInOUt;
  if (ref_dir.first * dirInOUt.first  + ref_dir.second * dirInOUt.second < 
      ref_dir.first * dirInOUt.second + ref_dir.second * dirInOUt.first) 
  {
      dirInOUt.first = temp_dir.second;
      dirInOUt.second = temp_dir.first;
  }
} 

std::size_t DPUtil::CounterOfPillar(const std::deque<PointPair>& dirs_stack) {
  std::size_t counter = 0;
  const PointPair pillar_dir = PointPair(Point3D(0,0,-1), Point3D(0,0,-1));
  for (const auto& p_pair : dirs_stack) {
    if (p_pair == pillar_dir) {
      counter ++;
    }
  }
  return counter;
}

Point3D DPUtil::RANSACPoisiton(const std::deque<Point3D>& pos_filter_stack, const float& margin, std::size_t& inlier_size) {
  inlier_size = 0;
  PointStack inlier_stack;
  for (const auto& p : pos_filter_stack) {
    std::size_t temp_inlier_size = 0;
    PointStack temp_inlier_stack;
    temp_inlier_stack.clear();
    for (const auto& cp : pos_filter_stack) {
      if ((p - cp).norm() < margin) {
        temp_inlier_stack.push_back(cp);
        temp_inlier_size ++;
      }
    }
    if (temp_inlier_size > inlier_size) {
      inlier_stack = temp_inlier_stack;
      inlier_size = temp_inlier_size;
    }
  }
  return DPUtil::AveragePoints(inlier_stack);
}

PointPair DPUtil::RANSACSurfDirs(const std::deque<PointPair>& surf_dirs_stack, const float& margin, std::size_t& inlier_size) {
  inlier_size = 0;
  std::vector<PointPair> inlier_stack;
  const std::size_t pillar_size = DPUtil::CounterOfPillar(surf_dirs_stack);
  const PointPair pillar_dir = PointPair(Point3D(0,0,-1), Point3D(0,0,-1));
  for (const auto& dir_pair : surf_dirs_stack) {
    if (dir_pair == pillar_dir) continue;
    std::size_t temp_inlier_size = 0;
    std::vector<PointPair> temp_inlier_stack;
    temp_inlier_stack.clear();
    for (const auto& cdir_pair : surf_dirs_stack) {
      if (cdir_pair == pillar_dir) continue;
      if (DPUtil::DirsDistance(dir_pair, cdir_pair) < margin) {
        temp_inlier_stack.push_back(cdir_pair);
        temp_inlier_size ++;
      }
    }
    if (temp_inlier_size > inlier_size) {
      inlier_stack = temp_inlier_stack;
      inlier_size = temp_inlier_size;
    }
  }
  if (pillar_size > inlier_size) { // this node should be a pillar
    inlier_size = pillar_size;
    return pillar_dir;
  }
  return DPUtil::AverageDirs(inlier_stack);
}

Point3D DPUtil::AveragePoints(const PointStack& point_stack) {
  Point3D mean_p(0,0,0);
  for (const auto& pos : point_stack) {
    mean_p = mean_p + pos;
  }
  return mean_p / (float)point_stack.size();
}

PointPair DPUtil::AverageDirs(const std::vector<PointPair>& dirs_stack) {
  const PointPair pillar_dir = PointPair(Point3D(0,0,-1), Point3D(0,0,-1));
  Point3D mean_dir1(0,0,0);
  Point3D mean_dir2(0,0,0);
  for (const auto& dir_pair : dirs_stack) {
    if (dir_pair == pillar_dir) continue;
    mean_dir1 = mean_dir1 + dir_pair.first;
    mean_dir2 = mean_dir2 + dir_pair.second; 
  }
  mean_dir1 = mean_dir1.normalize();
  mean_dir2 = mean_dir2.normalize();
  return PointPair(mean_dir1, mean_dir2);
}

void DPUtil::ConvertCTNodeStackToPCL(const CTNodeStack& ctnode_stack, 
                                     const PointCloudPtr& point_cloud_ptr)
{
  const std::size_t N = ctnode_stack.size();
  point_cloud_ptr->clear(), point_cloud_ptr->resize(N);
  for (std::size_t i=0; i<N; i++) {
    point_cloud_ptr->points[i] = DPUtil::Point3DToPCLPoint(ctnode_stack[i]->position);
  }
}

void DPUtil::CropBoxCloud(const PointCloudPtr& cloudInOut, 
                          const Point3D& center_p, 
                          const Point3D& crop_size) 
{
  pcl::CropBox<PCLPoint> boxFilter;
  pcl::PointCloud<PCLPoint> cropBox_cloud;
  Eigen::Vector4f min_vec(center_p.x - crop_size.x,
                          center_p.y - crop_size.y,
                          center_p.z - crop_size.z, 1.0);
  Eigen::Vector4f max_vec(center_p.x + crop_size.x,
                          center_p.y + crop_size.y,
                          center_p.z + crop_size.z, 1.0);
  boxFilter.setMin(min_vec), boxFilter.setMax(max_vec);
  boxFilter.setInputCloud(cloudInOut);
  boxFilter.filter(cropBox_cloud);
  *cloudInOut = cropBox_cloud;
}

void DPUtil::CreatePointsAroundCenter(const Point3D& center_p, 
                                      const float& radius,
                                      const float& resol,
                                      PointStack& points_stack,
                                      const bool& is_sort)
{
  const int H_SIZE = std::ceil(radius / resol);
  const std::size_t grid_size = (2*H_SIZE+1) * (2*H_SIZE+1);
  points_stack.clear(), points_stack.resize(grid_size);
  std::size_t idx = 0;
  for (int i=-H_SIZE; i<=H_SIZE; i++) {
      for (int j=-H_SIZE; j<=H_SIZE; j++) {
          Point3D p(center_p.x+resol*i, center_p.y+resol*j, center_p.z);
          p.intensity = (p - center_p).norm();
          points_stack[idx] = p;
          idx ++;
      }
  }
  points_stack.resize(idx);
  if (is_sort) {
    std::sort(points_stack.begin(), points_stack.end(), intensity_comp());
  }
}

bool DPUtil::IsVoteTrue(const std::deque<int>& votes) {
  const int N = votes.size();
  float sum = 0;
  for (const auto& v : votes) {
      sum += v;
  }
  if (sum > std::floor(N / 2.0)) {
      return true;
  }
  return false;
}


/************************** UNUSE CODE ****************************/

// PointPair DPUtil::TangentFreePoint(const NavNodePtr& from_node_ptr,
//                                    const NavNodePtr& to_node_ptr) 
// {
//   const Point3D from_p = from_node_ptr->position;
//   const Point3D to_p = to_node_ptr->position;
//   PointPair tangent_pair;
//   if (DPUtil::IsGroundOnly) {
//     const float radius = DPUtil::kNavClearDist;
//     const Point3D diff_p = to_p - from_p;
//     const float dist = diff_p.norm();
//     const float tan_dist = sqrt(dist*dist - radius*radius);
//     const float d_theta = std::asin(radius / dist);
//     const float ori_theta = DPUtil::ApproxAtan2(diff_p.y, diff_p.x);
//     float c_theta = ori_theta + d_theta;
//     const Point3D candiate_p1(from_p.x + tan_dist * cos(c_theta),
//                               from_p.y + tan_dist * sin(c_theta),
//                               (from_p.z + to_p.z) / 2.0);
//     c_theta = ori_theta - d_theta;
//     const Point3D candiate_p2(from_p.x + tan_dist * cos(c_theta),
//                               from_p.y + tan_dist * sin(c_theta),
//                               (from_p.z + to_p.z) / 2.0);
//     tangent_pair.first  = candiate_p1;                              
//     tangent_pair.second = candiate_p2;
//     tangent_pair.first.intensity = 0.0, tangent_pair.second.intensity = 0.0;

//     if (DPUtil::IsPointInFreeSpace(candiate_p1)) tangent_pair.first.intensity = 1.0;
//     if (DPUtil::IsPointInFreeSpace(candiate_p2)) tangent_pair.second.intensity = 1.0;
//     if (tangent_pair.first.intensity == 0.0 && tangent_pair.second.intensity == 0.0) {
//       Point3D connect_p = to_p + (from_p - to_p).normalize() * (DPUtil::kNavClearDist);
//       connect_p.intensity = 0.0;
//       tangent_pair.first = connect_p, tangent_pair.second = connect_p;
//     }
//     return tangent_pair;
//   } else {
//     //TODO!
//     ROS_ERROR("Tangent Point in 3D has not been implemented yet.");
//     return PointPair(Point3D(0,0,0), Point3D(0,0,0));
//   } 
// }