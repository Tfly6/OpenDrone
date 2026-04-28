/*
 * Dynamic Route Planner
 * Copyright (C) 2021 Fan Yang - All rights reserved
 * fanyang2@andrew.cmu.edu,   
 */

#include "airfar_planner/contour_detector.h"

// const static int BLUR_SIZE = 10;

/***************************************************************************************/

void ContourDetector::Init(const ContourDetectParams& params) {
    cd_params_ = params;
    /* Allocate Pointcloud pointer memory */
    new_corners_cloud_   = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
    // Init projection cv Mat
    MAT_SIZE = std::ceil((cd_params_.sensor_range * 2.0 + DPUtil::kCellLength) / cd_params_.voxel_dim);
    MAT_RESIZE = MAT_SIZE * (int)cd_params_.kRatio;
    CMAT = MAT_SIZE / 2, CMAT_RESIZE = MAT_RESIZE / 2;
    img_mat_ = cv::Mat::zeros(MAT_SIZE, MAT_SIZE, CV_32FC1);
    img_counter_ = 0;
    odom_node_ptr_ = NULL;
    refined_contours_.clear(), refined_hierarchy_.clear();
    DIST_LIMIT = cd_params_.kRatio * 3.0;
    ALIGN_ANGLE_COS = cos(DPUtil::kAcceptAlign);
    VOXEL_DIM_INV = 1.0 / cd_params_.voxel_dim;
}

void ContourDetector::BuildTerrianImgAndExtractContour(const NavNodePtr& odom_node_ptr,
                                                       const float& layer_height,
                                                       const PointCloudPtr& surround_cloud,
                                                       std::vector<PointStack>& realworl_contour) {
    CVPointStack cv_corners;
    PointStack corner_vec;
    this->UpdateOdom(odom_node_ptr, layer_height);
    this->ResetImgMat(img_mat_);
    this->UpdateImgMatWithCloud(surround_cloud, img_mat_);
    this->ExtractContourFromImg(img_mat_, refined_contours_, realworl_contour);
}

void ContourDetector::UpdateImgMatWithCloud(const PointCloudPtr& pc, cv::Mat& img_mat) {
    int row_idx, col_idx;
    int inf_row, inf_col;
    const float THRED = DPUtil::IsSimulation ? std::floor(cd_params_.kThredValue/2.0) : cd_params_.kThredValue;
    const std::vector<int> inflate_vec{-1, 0, 1};
    for (const auto& pcl_p : pc->points) {
        this->PointToImgSub(pcl_p, odom_pos_, row_idx, col_idx, false, false);
        if (!this->IsIdxesInImg(row_idx, col_idx)) continue;
        if (cd_params_.is_inflate) {
            for (const auto& dr : inflate_vec) {
                for (const auto& dc : inflate_vec) {
                    inf_row = row_idx+dr, inf_col = col_idx+dc;
                    if (this->IsIdxesInImg(inf_row, inf_col)) {
                        img_mat.at<float>(inf_row, inf_col) += 1.0;
                    }
                }
            }
        } else {
            img_mat.at<float>(row_idx, col_idx) += 1.0;
        }
    }
    cv::threshold(img_mat, img_mat, THRED, 1.0, cv::ThresholdTypes::THRESH_BINARY);
    // if (cd_params_.is_save_img) {
    //     this->SaveCurrentImg(img_mat);
    // }
}

void ContourDetector::ResizeAndBlurImg(const cv::Mat& img, cv::Mat& Rimg) {
    img.convertTo(Rimg, CV_8UC1, 255);
    cv::morphologyEx(Rimg, Rimg, cv::MORPH_CLOSE, getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));
    cv::morphologyEx(Rimg, Rimg, cv::MORPH_OPEN, getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
    cv::resize(Rimg, Rimg, cv::Size(), cd_params_.kRatio, cd_params_.kRatio, 
               cv::InterpolationFlags::INTER_LINEAR);

    cv::boxFilter(Rimg, Rimg, -1, cv::Size(cd_params_.kBlurSize, cd_params_.kBlurSize), cv::Point2i(-1, -1), false);

    // do close operation on Rimg    
    cv::morphologyEx(Rimg, Rimg, cv::MORPH_CLOSE, getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7)));
}

void ContourDetector::ExtractContourFromImg(const cv::Mat& img,
                                            std::vector<CVPointStack>& img_contours, 
                                            std::vector<PointStack>& realworld_contour)
{
    cv::Mat Rimg;
    this->ResizeAndBlurImg(img, Rimg);
    if (Rimg.size().height != MAT_RESIZE) {
        ROS_WARN("CD: resize image error.");
    }
    this->ExtractRefinedContours(Rimg, img_contours);
    this->ConvertContoursToRealWorld(img_contours, realworld_contour);
}

void ContourDetector::ConvertContoursToRealWorld(const std::vector<CVPointStack>& ori_contours,
                                                 std::vector<PointStack>& realWorld_contours)
{
    const std::size_t C_N = ori_contours.size();
    if (C_N == 0) return;
    realWorld_contours.resize(C_N);
    for (std::size_t i=0; i<C_N; i++) {
        const CVPointStack cv_contour = ori_contours[i];
        this->ConvertCVToPoint3DVector(cv_contour, realWorld_contours[i], true);
    }
}

void ContourDetector::ExtractRefinedContours(const cv::Mat& imgIn,
                                            std::vector<CVPointStack>& refined_contours) 
{ 
    
    std::vector<std::vector<cv::Point2i>> raw_contours;
    std::vector<std::vector<cv::Point2i>> raw_contours_vis;
    std::vector<std::vector<cv::Point2i>> approx_contours;
    refined_contours.clear(), refined_hierarchy_.clear();
    cv::findContours(imgIn, raw_contours, refined_hierarchy_, 
                     cv::RetrievalModes::RETR_TREE, 
                     cv::ContourApproximationModes::CHAIN_APPROX_TC89_L1);

    int raw_cnt = 0, approx_cnt = 0;
    for (std::size_t i=0; i<raw_contours.size(); i++) {
        const auto c = raw_contours[i];
        double peri = cv::arcLength(c, true);
        std::vector<cv::Point> approx;
        cv::approxPolyDP(c, approx, std::min(cd_params_.approx_eps * peri, double(DIST_LIMIT)), true);
        cv::approxPolyDP(approx, approx, std::max(cd_params_.approx_eps * peri, double(DIST_LIMIT)), true);
        // if (std::abs(cv::contourArea(approx) - cv::contourArea(c))/cv::contourArea(c) > 0.05) 
        //     cout<<"contour refine ratio: "<<std::abs(cv::contourArea(approx) - cv::contourArea(c))/cv::contourArea(c)<<endl;
        // ROS_WARN("CD: contour para diff: %f", cd_params_.approx_eps * peri - double(DIST_LIMIT));

        approx_contours.push_back(approx);

        raw_cnt += c.size();
        approx_cnt += approx.size();
    }
    // ROS_INFO("CD: raw contours size: %d, approx contours size: %d", raw_cnt, approx_cnt);
    raw_contours_vis = raw_contours;
    raw_contours = approx_contours;

    // std::vector<CVPointStack> inserted_contours;
    // this->CopyContours(raw_contours, inserted_contours);
    // cout<<"raw_contours size: "<<inserted_contours.size()<<endl;
    // for (std::size_t i=0; i<raw_contours.size(); i++) {
    //     const auto c = raw_contours[i];
    //     int inserted_cnt = 0;
    //     if (c.size() < 3) continue;
    //     for (std::size_t j=1; j<c.size(); j++) {
    //         cv::Point2f p = c[j];
    //         if (DPUtil::PixelDistance(c[j-1], p) > 4.0 * DIST_LIMIT) {
    //             cv::Point2f mid_p;
    //             mid_p.x = (p.x + c[j-1].x) / 2;
    //             mid_p.y = (p.y + c[j-1].y) / 2;
    //             inserted_contours[i].insert(inserted_contours[i].begin() + j + inserted_cnt, mid_p);      
    //             inserted_cnt ++;
    //         }
    //     }
    //     cout<<"inserted_contours size: "<<inserted_contours.size()<<endl;
    //     if (DPUtil::PixelDistance(c[c.size()-1], c[0]) > 4.0 * DIST_LIMIT) {
    //         cv::Point2f mid_p;
    //         mid_p.x = (c[0].x + c[c.size()-1].x) / 2;
    //         mid_p.y = (c[0].y + c[c.size()-1].y) / 2;
    //         inserted_contours[i].push_back(mid_p);     
    //     }
    // }
    // // inserted_contours.
    // this->RoundContours(inserted_contours, raw_contours);
    // cout<<"raw_contours size: "<<inserted_contours.size()<<endl;

    this->CopyContours(raw_contours, refined_contours);
    for (std::size_t i=0; i<raw_contours.size(); i++) {
        const auto c = raw_contours[i];
        const std::size_t c_size = c.size();
        std::size_t refined_idx = 0;
        for (std::size_t j=0; j<c_size; j++) {
            cv::Point2f p = c[j]; 
            if (refined_idx < 1 || DPUtil::PixelDistance(refined_contours[i][refined_idx-1], p) > DIST_LIMIT) {
                /** Reduce wall nodes */
                RemoveWallConnection(refined_contours[i], p, refined_idx);
                refined_contours[i][refined_idx] = p;
                refined_idx ++;
            }
        }
        RemoveWallConnection(refined_contours[i], refined_contours[i][0], refined_idx);
        refined_contours[i].resize(refined_idx);
    }
    this->TopoFilterContours(refined_contours);

    // visulize refined_contours
    if (cd_params_.is_save_img) {
        cv::Mat img_raw_contours;
        imgIn.copyTo(img_raw_contours);
        cv::cvtColor(img_raw_contours, img_raw_contours, cv::COLOR_GRAY2BGR);

        cv::Mat img_refined_contours;
        img_raw_contours.copyTo(img_refined_contours);

        cv::Mat img_concat;

        for (int i=0; i<raw_contours_vis.size(); i++) {
            const auto poly = raw_contours_vis[i];
            for (int j=0; j<poly.size(); j++) {
                cv::circle(img_raw_contours, poly[j], 3, cv::Scalar(0, 0, 255), 1);
            }
        }
        for (int i=0; i<refined_contours.size(); i++) {
            const auto poly = refined_contours[i];
            for (int j=0; j<poly.size(); j++) {
                cv::circle(img_refined_contours, poly[j], 3, cv::Scalar(0, 0, 255), 1);
            }
        }

        cv::resize(img_raw_contours, img_raw_contours, cv::Size(), 1.0/cd_params_.kRatio, 1.0/cd_params_.kRatio, 
            cv::InterpolationFlags::INTER_LINEAR);
        cv::resize(img_refined_contours, img_refined_contours, cv::Size(), 1.0/cd_params_.kRatio, 1.0/cd_params_.kRatio, 
            cv::InterpolationFlags::INTER_LINEAR);
        cv::hconcat(img_raw_contours, img_refined_contours, img_concat);
        this->SaveCurrentImg(img_concat);
    }
}

void ContourDetector::TopoFilterContours(std::vector<CVPointStack>& contoursInOut) {
    std::unordered_set<int> remove_idxs;
    for (int i=0; i<contoursInOut.size(); i++) {
        if (remove_idxs.find(i) != remove_idxs.end()) continue;
        const auto poly = contoursInOut[i];
        if (poly.size() < 3) {
            remove_idxs.insert(i);
        } else if (!DPUtil::PointInsideAPoly(poly, free_odom_resized_)) {
            InternalContoursIdxs(refined_hierarchy_, i, remove_idxs);
        }
    }
    std::vector<CVPointStack> temp_contours = contoursInOut;
    contoursInOut.clear();
    for (int i=0; i<temp_contours.size(); i++) {
        if (remove_idxs.find(i) != remove_idxs.end()) continue;
        contoursInOut.push_back(temp_contours[i]);
    }
}

/************************** UNUSE CODE ****************************/

// float ContourDetector::ComputeConvexityValue(const std::size_t idx, 
//                                             const CVPointStack contour,
//                                             const cv::Mat& imgIn)
// {
//     const std::size_t N = contour.size();
//     const cv::Point2i c_p = contour[idx];
//     const cv::Point2i ref_p1 = contour[(idx+N_STEP)%N];
//     const cv::Point2i ref_p2 = contour[(idx-N_STEP)%N];
//     cv::Point2i ref_pc;
//     ref_pc.x = (int)std::round((ref_p1.x + ref_p2.x) / 2.0);
//     ref_pc.y = (int)std::round((ref_p1.y + ref_p2.y) / 2.0);
//     const int img_value = (int)imgIn.at<uchar>(ref_pc.y, ref_pc.x);
//     float c_dist = std::hypotf(c_p.x - ref_pc.x, c_p.y - ref_pc.y);
//     if (img_value < OBSVALUE) {
//         c_dist = -c_dist;
//     }
//     return c_dist;
// }


// void ContourDetector::ExtractConersFromImg(const cv::Mat& img, 
//                                           CVPointStack& cv_corners)
// {
//     cv_corners.clear();
//     cv::Mat mask;
//     const double kMinDistance = cd_params_.near_dist * 1.1 / cd_params_.voxel_dim;
//     const bool useHarrisDetector = false;
//     const double k = 0.04;

//     cv::goodFeaturesToTrack(img, cv_corners, 
//                             cd_params_.kMaxCorners, 
//                             cd_params_.kQualityLevel, 
//                             kMinDistance, mask, 
//                             cd_params_.kBlockSize, 
//                             useHarrisDetector, k);
// }

// bool ContourDetector::IsAdjacentVertex(const CVPointStack& contour,
//                                       const std::size_t& cur_idx,
//                                       const std::size_t& horizon,
//                                       std::size_t& adjacent_id) 
// {
//     const int c_size = contour.size();
//     if (c_size < 3 || horizon < 3) return false;
//     const auto cur_p = contour[cur_idx];
//     for (std::size_t i=2; i<horizon; i++) {
//         const int ref_idx = cur_idx + i;
//         if (ref_idx == c_size) break;
//         const auto ref_p = contour[ref_idx];
//         if (DPUtil::PixelDistance(cur_p, ref_p) < DIST_LIMIT) {
//             adjacent_id = ref_idx;
//             return true;
//         }
//     }
//     return false;
// }

// void ContourDetector::UpdateVerticesCloud(const PointStack& vertices_stack) {
//     DPUtil::ConvertCloudToPCL(vertices_stack, new_corners_cloud_);
//     ROS_INFO("CD: number of raw corners detected: %d", new_corners_cloud_->size());
//     for (std::size_t i=0; i<new_corners_cloud_->size(); i++) {
//         const Point3D p(new_corners_cloud_->points[i]);
//         const Point3D refined_p = CPerceptor::PreRefineCornerPosition(p);
//         new_corners_cloud_->points[i] = DPUtil::Point3DToPCLPoint(refined_p);
//     }
//     DPUtil::FilterCloud(new_corners_cloud_, DPUtil::kLeafSize * 2.0);
//     ROS_INFO("CD: corners updated, number of new corners: %d", new_corners_cloud_->size());
// }



