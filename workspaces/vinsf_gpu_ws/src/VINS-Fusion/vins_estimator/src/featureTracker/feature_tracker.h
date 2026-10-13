/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#pragma once

#include <cstdio>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <queue>
#include <execinfo.h>
#include <csignal>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include <opencv2/cudaoptflow.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/core/cuda.hpp>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"
#include "../estimator/parameters.h"
#include "../utility/tic_toc.h"

using namespace std;
using namespace camodocal;
using namespace Eigen;

bool inBorder(const cv::Point2f &pt);
void reduceVector(vector<cv::Point2f> &v, vector<uchar> status);
void reduceVector(vector<int> &v, vector<uchar> status);

class FeatureTracker
{
public:
    FeatureTracker();
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> trackImage(double _cur_time, const cv::Mat &_img, const cv::Mat &_img1 = cv::Mat());
    void setMask();
    void addPoints();
    void clearState();
    void readIntrinsicParameter(const vector<string> &calib_file);
    void showUndistortion(const string &name);
    void rejectWithF();
    void undistortedPoints();
    vector<cv::Point2f> undistortedPts(vector<cv::Point2f> &pts, camodocal::CameraPtr cam);
    vector<cv::Point2f> ptsVelocity(vector<int> &ids, vector<cv::Point2f> &pts, 
                                    map<int, cv::Point2f> &cur_id_pts, map<int, cv::Point2f> &prev_id_pts);
    void showTwoImage(const cv::Mat &img1, const cv::Mat &img2, 
                      vector<cv::Point2f> pts1, vector<cv::Point2f> pts2);
    void drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight, 
                                   vector<int> &curLeftIds,
                                   vector<cv::Point2f> &curLeftPts, 
                                   vector<cv::Point2f> &curRightPts,
                                   map<int, cv::Point2f> &prevLeftPtsMap);
    void setPrediction(map<int, Eigen::Vector3d> &predictPts);
    double distance(cv::Point2f &pt1, cv::Point2f &pt2);
    void removeOutliers(set<int> &removePtsIds);
    cv::Mat getTrackImage();
    bool inBorder(const cv::Point2f &pt);

private:
    void initGpuResources(const cv::Mat &image);
    int countGpuStatus(const cv::cuda::GpuMat &status);
    int compactGpuFlow(const cv::cuda::GpuMat &current_points,
                       const cv::cuda::GpuMat &forward_status,
                       const cv::cuda::GpuMat &previous_points,
                       const cv::cuda::GpuMat &reverse_points,
                       const cv::cuda::GpuMat &reverse_status,
                       bool use_reverse,
                       int point_count,
                       vector<cv::Point2f> &compact_points,
                       vector<int> &compact_indices);
    void uploadPointVector(const vector<cv::Point2f> &points,
                           cv::cuda::GpuMat &buffer);

    int row, col;
    cv::Mat imTrack;
    cv::Mat mask;
    cv::Mat fisheye_mask;
    cv::Mat prev_img, cur_img;
    vector<cv::Point2f> n_pts;
    int sum_n;
    vector<cv::Point2f> predict_pts;
    vector<cv::Point2f> predict_pts_debug;
    vector<cv::Point2f> prev_pts, cur_pts, cur_right_pts;
    vector<cv::Point2f> prev_un_pts, cur_un_pts, cur_un_right_pts;
    vector<cv::Point2f> pts_velocity, right_pts_velocity;
    vector<int> ids, ids_right;
    vector<int> track_cnt;
    map<int, cv::Point2f> cur_un_pts_map, prev_un_pts_map;
    map<int, cv::Point2f> cur_un_right_pts_map, prev_un_right_pts_map;
    map<int, cv::Point2f> prevLeftPtsMap;
    vector<camodocal::CameraPtr> m_camera;
    double cur_time;
    double prev_time;
    bool stereo_cam;
    int n_id;
    bool hasPrediction;

    cv::Size gpu_image_size;
    int gpu_image_type;
    cv::cuda::GpuMat gpu_prev_img;
    cv::cuda::GpuMat gpu_cur_img;
    cv::cuda::GpuMat gpu_right_img;
    cv::cuda::GpuMat gpu_prev_pts;
    cv::cuda::GpuMat gpu_cur_pts;
    cv::cuda::GpuMat gpu_reverse_pts;
    cv::cuda::GpuMat gpu_status;
    cv::cuda::GpuMat gpu_reverse_status;
    cv::cuda::GpuMat gpu_detected_pts;
    cv::cuda::GpuMat gpu_mask;
    cv::cuda::GpuMat gpu_compact_pts;
    cv::cuda::GpuMat gpu_compact_indices;
    cv::cuda::GpuMat gpu_compact_count;
    cv::cuda::GpuMat gpu_un_pts;
    cv::cuda::HostMem host_compact_pts;
    cv::cuda::HostMem host_compact_indices;
    cv::cuda::HostMem host_compact_count;
    cv::cuda::HostMem host_detected_pts;
    cv::cuda::HostMem host_un_pts;
    cv::cuda::HostMem host_mask;
    vector<int> gpu_valid_indices;
    vector<cv::Point2f> gpu_old_prev_pts;
    vector<int> gpu_old_ids;
    vector<int> gpu_old_track_cnt;
    vector<pair<int, pair<cv::Point2f, int>>> mask_candidates;
    cv::Ptr<cv::cuda::SparsePyrLKOpticalFlow> gpu_flow_level3;
    cv::Ptr<cv::cuda::SparsePyrLKOpticalFlow> gpu_flow_level1_initial;
    cv::Ptr<cv::cuda::CornersDetector> gpu_detector;
    cv::cuda::Stream gpu_stream;
    std::mutex state_mutex;

    // Optional low-rate profiling counters. They are only updated when
    // GPU_PROFILE is enabled and do not affect the tracking path.
    uint64_t profile_frames;
    double profile_frame_ms;
    double profile_temporal_ms;
    double profile_feature_ms;
    double profile_undistort_ms;
    double profile_stereo_ms;
    double profile_upload_ms;
    double profile_download_ms;
    double profile_wait_ms;
};
