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

#include "feature_tracker.h"
#include "feature_tracker_cuda.h"

#include <algorithm>
#include <cmath>

bool FeatureTracker::inBorder(const cv::Point2f &pt)
{
    const int BORDER_SIZE = 1;
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < col - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < row - BORDER_SIZE;
}

double distance(cv::Point2f pt1, cv::Point2f pt2)
{
    //printf("pt1: %f %f pt2: %f %f\n", pt1.x, pt1.y, pt2.x, pt2.y);
    double dx = pt1.x - pt2.x;
    double dy = pt1.y - pt2.y;
    return sqrt(dx * dx + dy * dy);
}

void reduceVector(vector<cv::Point2f> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

void reduceVector(vector<int> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

FeatureTracker::FeatureTracker()
{
    stereo_cam = 0;
    n_id = 0;
    hasPrediction = false;
    sum_n = 0;
    gpu_image_type = -1;
    profile_frames = 0;
    profile_frame_ms = 0.0;
    profile_temporal_ms = 0.0;
    profile_feature_ms = 0.0;
    profile_undistort_ms = 0.0;
    profile_stereo_ms = 0.0;
    profile_upload_ms = 0.0;
    profile_download_ms = 0.0;
    profile_wait_ms = 0.0;
}

void FeatureTracker::clearState()
{
    std::lock_guard<std::mutex> lock(state_mutex);
    gpu_stream.waitForCompletion();
    prev_img.release();
    cur_img.release();
    imTrack.release();
    mask.release();
    n_pts.clear();
    predict_pts.clear();
    predict_pts_debug.clear();
    prev_pts.clear();
    cur_pts.clear();
    cur_right_pts.clear();
    prev_un_pts.clear();
    cur_un_pts.clear();
    cur_un_right_pts.clear();
    pts_velocity.clear();
    right_pts_velocity.clear();
    ids.clear();
    ids_right.clear();
    track_cnt.clear();
    cur_un_pts_map.clear();
    prev_un_pts_map.clear();
    cur_un_right_pts_map.clear();
    prev_un_right_pts_map.clear();
    prevLeftPtsMap.clear();
    gpu_valid_indices.clear();
    gpu_old_prev_pts.clear();
    gpu_old_ids.clear();
    gpu_old_track_cnt.clear();
    mask_candidates.clear();
    n_id = 0;
    cur_time = 0.0;
    prev_time = 0.0;
    hasPrediction = false;
}

void FeatureTracker::initGpuResources(const cv::Mat &image)
{
    if ((!USE_GPU && !USE_GPU_ACC_FLOW) || image.empty())
        return;

    if (gpu_image_size != image.size() || gpu_image_type != image.type() ||
        gpu_flow_level3.empty() || gpu_flow_level1_initial.empty() || gpu_detector.empty())
    {
        gpu_stream.waitForCompletion();
        gpu_image_size = image.size();
        gpu_image_type = image.type();
        gpu_prev_img.release();
        gpu_cur_img.release();
        gpu_right_img.release();
        gpu_prev_pts.release();
        gpu_cur_pts.release();
        gpu_reverse_pts.release();
        gpu_status.release();
        gpu_reverse_status.release();
        gpu_detected_pts.release();
        gpu_mask.release();
        gpu_compact_pts.release();
        gpu_compact_indices.release();
        gpu_compact_count.release();
        gpu_un_pts.release();
        host_compact_pts.release();
        host_compact_indices.release();
        host_compact_count.release();
        host_detected_pts.release();
        host_un_pts.release();
        host_mask.release();
        gpu_flow_level3 = cv::cuda::SparsePyrLKOpticalFlow::create(cv::Size(21, 21), 3, 30, false);
        gpu_flow_level1_initial = cv::cuda::SparsePyrLKOpticalFlow::create(cv::Size(21, 21), 1, 30, true);
        gpu_detector = cv::cuda::createGoodFeaturesToTrackDetector(image.type(), std::max(1, MAX_CNT), 0.01, MIN_DIST);
        const int max_points = std::max(1, MAX_CNT);
        gpu_compact_pts.create(max_points, 1, CV_32FC2);
        gpu_compact_indices.create(max_points, 1, CV_32SC1);
        gpu_compact_count.create(1, 1, CV_32SC1);
        host_compact_pts.create(max_points, 1, CV_32FC2);
        host_compact_indices.create(max_points, 1, CV_32SC1);
        host_compact_count.create(1, 1, CV_32SC1);
        host_detected_pts.create(max_points, 1, CV_32FC2);
        gpu_un_pts.create(max_points, 1, CV_32FC2);
        host_un_pts.create(max_points, 1, CV_32FC2);
        host_mask.create(image.rows, image.cols, CV_8UC1);
        gpu_prev_pts.create(1, max_points, CV_32FC2);
        gpu_cur_pts.create(1, max_points, CV_32FC2);
        gpu_reverse_pts.create(1, max_points, CV_32FC2);
        gpu_status.create(1, max_points, CV_8UC1);
        gpu_reverse_status.create(1, max_points, CV_8UC1);
        gpu_mask.create(image.rows, image.cols, CV_8UC1);
        gpu_valid_indices.reserve(max_points);
        gpu_old_prev_pts.reserve(max_points);
        gpu_old_ids.reserve(max_points);
        gpu_old_track_cnt.reserve(max_points);
        mask_candidates.reserve(max_points);
    }
    if (GPU_PROFILE)
    {
        TicToc profile_timer;
        gpu_cur_img.upload(image, gpu_stream);
        profile_upload_ms += profile_timer.toc();
    }
    else
        gpu_cur_img.upload(image, gpu_stream);
}

void FeatureTracker::uploadPointVector(const vector<cv::Point2f> &points,
                                       cv::cuda::GpuMat &buffer)
{
    if (points.empty())
        return;

    if (GPU_PROFILE)
    {
        TicToc profile_timer;
        buffer.upload(points, gpu_stream);
        profile_upload_ms += profile_timer.toc();
    }
    else
        buffer.upload(points, gpu_stream);
}

int FeatureTracker::countGpuStatus(const cv::cuda::GpuMat &status)
{
    if (status.empty())
        return 0;

    cv::cuda::countNonZero(status, gpu_compact_count, gpu_stream);
    if (GPU_PROFILE)
    {
        TicToc profile_timer;
        gpu_compact_count.download(host_compact_count.createMatHeader(), gpu_stream);
        profile_download_ms += profile_timer.toc();
        profile_timer.tic();
        gpu_stream.waitForCompletion();
        profile_wait_ms += profile_timer.toc();
    }
    else
    {
        gpu_compact_count.download(host_compact_count.createMatHeader(), gpu_stream);
        gpu_stream.waitForCompletion();
    }
    return host_compact_count.createMatHeader().at<int>(0, 0);
}

int FeatureTracker::compactGpuFlow(const cv::cuda::GpuMat &current_points,
                                   const cv::cuda::GpuMat &forward_status,
                                   const cv::cuda::GpuMat &previous_points,
                                   const cv::cuda::GpuMat &reverse_points,
                                   const cv::cuda::GpuMat &reverse_status,
                                   bool use_reverse,
                                   int point_count,
                                   vector<cv::Point2f> &compact_points,
                                   vector<int> &compact_indices)
{
    compact_points.clear();
    compact_indices.clear();
    if (point_count <= 0)
        return 0;

    if (!vins_cuda::compactFlow(current_points, forward_status, previous_points,
                                reverse_points, reverse_status, use_reverse,
                                col, row, 0.5f, gpu_compact_pts,
                                gpu_compact_indices, gpu_compact_count,
                                gpu_stream))
    {
        ROS_ERROR("CUDA flow compaction could not be queued");
        return -1;
    }

    cv::Mat host_points = host_compact_pts.createMatHeader().rowRange(0, point_count);
    cv::Mat host_indices = host_compact_indices.createMatHeader().rowRange(0, point_count);
    if (GPU_PROFILE)
    {
        TicToc profile_timer;
        gpu_compact_pts.rowRange(0, point_count).download(host_points, gpu_stream);
        gpu_compact_indices.rowRange(0, point_count).download(host_indices, gpu_stream);
        gpu_compact_count.download(host_compact_count.createMatHeader(), gpu_stream);
        profile_download_ms += profile_timer.toc();
        profile_timer.tic();
        gpu_stream.waitForCompletion();
        profile_wait_ms += profile_timer.toc();
    }
    else
    {
        gpu_compact_pts.rowRange(0, point_count).download(host_points, gpu_stream);
        gpu_compact_indices.rowRange(0, point_count).download(host_indices, gpu_stream);
        gpu_compact_count.download(host_compact_count.createMatHeader(), gpu_stream);
        gpu_stream.waitForCompletion();
    }

    int valid_count = host_compact_count.createMatHeader().at<int>(0, 0);
    valid_count = std::max(0, std::min(valid_count, point_count));
    const cv::Point2f *points_ptr = host_points.ptr<cv::Point2f>();
    const int *indices_ptr = host_indices.ptr<int>();
    for (int i = 0; i < valid_count; ++i)
    {
        if (indices_ptr[i] < 0 || indices_ptr[i] >= point_count ||
            !std::isfinite(points_ptr[i].x) || !std::isfinite(points_ptr[i].y))
        {
            ROS_ERROR("CUDA flow compaction returned invalid point index/data");
            compact_points.clear();
            compact_indices.clear();
            return -1;
        }
    }
    // The CUDA compaction kernel uses atomicAdd, so the order in which
    // threads append valid points is otherwise nondeterministic.  Feature
    // order affects the later distance-mask tie break; restore the original
    // input order here to keep GPU runs reproducible without changing which
    // points passed the optical-flow checks.
    vector<pair<int, cv::Point2f>> ordered_points;
    ordered_points.reserve(valid_count);
    for (int i = 0; i < valid_count; ++i)
        ordered_points.emplace_back(indices_ptr[i], points_ptr[i]);
    sort(ordered_points.begin(), ordered_points.end(),
         [](const pair<int, cv::Point2f> &a, const pair<int, cv::Point2f> &b)
         {
             return a.first < b.first;
         });
    compact_points.reserve(valid_count);
    compact_indices.reserve(valid_count);
    for (const auto &item : ordered_points)
    {
        compact_indices.push_back(item.first);
        compact_points.push_back(item.second);
    }
    return valid_count;
}

void FeatureTracker::setMask()
{
    if (USE_GPU || USE_GPU_ACC_FLOW)
    {
        if (host_mask.empty() || host_mask.rows != row || host_mask.cols != col)
            host_mask.create(row, col, CV_8UC1);
        mask = host_mask.createMatHeader();
        mask.setTo(cv::Scalar(255));
    }
    else
        mask = cv::Mat(row, col, CV_8UC1, cv::Scalar(255));

    // prefer to keep features that are tracked for long time
    mask_candidates.clear();
    if (mask_candidates.capacity() < cur_pts.size())
        mask_candidates.reserve(cur_pts.size());

    for (unsigned int i = 0; i < cur_pts.size(); i++)
        mask_candidates.push_back(make_pair(track_cnt[i], make_pair(cur_pts[i], ids[i])));

    sort(mask_candidates.begin(), mask_candidates.end(), [](const pair<int, pair<cv::Point2f, int>> &a, const pair<int, pair<cv::Point2f, int>> &b)
         {
            return a.first > b.first;
         });

    cur_pts.clear();
    ids.clear();
    track_cnt.clear();

    for (auto &it : mask_candidates)
    {
        if (mask.at<uchar>(it.second.first) == 255)
        {
            cur_pts.push_back(it.second.first);
            ids.push_back(it.second.second);
            track_cnt.push_back(it.first);
            cv::circle(mask, it.second.first, MIN_DIST, 0, -1);
        }
    }
}

void FeatureTracker::addPoints()
{
    for (auto &p : n_pts)
    {
        cur_pts.push_back(p);
        ids.push_back(n_id++);
        track_cnt.push_back(1);
    }
}

double FeatureTracker::distance(cv::Point2f &pt1, cv::Point2f &pt2)
{
    //printf("pt1: %f %f pt2: %f %f\n", pt1.x, pt1.y, pt2.x, pt2.y);
    double dx = pt1.x - pt2.x;
    double dy = pt1.y - pt2.y;
    return sqrt(dx * dx + dy * dy);
}

map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> FeatureTracker::trackImage(double _cur_time, const cv::Mat &_img, const cv::Mat &_img1)
{
    std::lock_guard<std::mutex> state_lock(state_mutex);
    TicToc t_r;
    cur_time = _cur_time;
    cur_img = _img;
    row = cur_img.rows;
    col = cur_img.cols;
    cv::Mat rightImg = _img1;
    const size_t max_points = static_cast<size_t>(std::max(1, MAX_CNT));
    if (prev_pts.capacity() < max_points)
    {
        prev_pts.reserve(max_points);
        cur_pts.reserve(max_points);
        cur_right_pts.reserve(max_points);
        prev_un_pts.reserve(max_points);
        cur_un_pts.reserve(max_points);
        cur_un_right_pts.reserve(max_points);
        pts_velocity.reserve(max_points);
        right_pts_velocity.reserve(max_points);
        ids.reserve(max_points);
        ids_right.reserve(max_points);
        track_cnt.reserve(max_points);
        n_pts.reserve(max_points);
    }
    initGpuResources(cur_img);
    if ((USE_GPU || USE_GPU_ACC_FLOW) && !rightImg.empty())
    {
        if (GPU_PROFILE)
        {
            TicToc profile_timer;
            gpu_right_img.upload(rightImg, gpu_stream);
            profile_upload_ms += profile_timer.toc();
        }
        else
            gpu_right_img.upload(rightImg, gpu_stream);
    }
    /*
    {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        clahe->apply(cur_img, cur_img);
        if(!rightImg.empty())
            clahe->apply(rightImg, rightImg);
    }
    */
    cur_pts.clear();

    TicToc temporal_profile_timer;
    if (prev_pts.size() > 0)
    {
        vector<uchar> status;
        if(!USE_GPU_ACC_FLOW)
        {
            TicToc t_o;
            
            vector<float> err;
            if(hasPrediction)
            {
                cur_pts = predict_pts;
                cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21), 1, 
                cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);
                
                int succ_num = 0;
                for (size_t i = 0; i < status.size(); i++)
                {
                    if (status[i])
                        succ_num++;
                }
                if (succ_num < 10)
                cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21), 3);
            }
            else
                cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21), 3);
            // reverse check
            if(FLOW_BACK)
            {
                vector<uchar> reverse_status;
                vector<cv::Point2f> reverse_pts = prev_pts;
                cv::calcOpticalFlowPyrLK(cur_img, prev_img, cur_pts, reverse_pts, reverse_status, err, cv::Size(21, 21), 1, 
                cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);
                //cv::calcOpticalFlowPyrLK(cur_img, prev_img, cur_pts, reverse_pts, reverse_status, err, cv::Size(21, 21), 3); 
                for(size_t i = 0; i < status.size(); i++)
                {
                    if(status[i] && reverse_status[i] && distance(prev_pts[i], reverse_pts[i]) <= 0.5)
                    {
                        status[i] = 1;
                    }
                    else
                        status[i] = 0;
                }
            }
            // printf("temporal optical flow costs: %fms\n", t_o.toc());
        }
        else
        {
            TicToc t_og;
            uploadPointVector(prev_pts, gpu_prev_pts);
            if(hasPrediction)
            {
                uploadPointVector(predict_pts, gpu_cur_pts);
                gpu_flow_level1_initial->calc(gpu_prev_img, gpu_cur_img, gpu_prev_pts, gpu_cur_pts, gpu_status, cv::noArray(), gpu_stream);
                if (countGpuStatus(gpu_status) < 10)
                    gpu_flow_level3->calc(gpu_prev_img, gpu_cur_img, gpu_prev_pts, gpu_cur_pts, gpu_status, cv::noArray(), gpu_stream);
            }
            else
            {
                // gpu_flow_level3 was created with useInitialFlow=false, so
                // its destination does not need an initial host upload.
                gpu_flow_level3->calc(gpu_prev_img, gpu_cur_img, gpu_prev_pts, gpu_cur_pts, gpu_status, cv::noArray(), gpu_stream);
            }
            if(FLOW_BACK)
            {
                uploadPointVector(prev_pts, gpu_reverse_pts);
                gpu_flow_level1_initial->calc(gpu_cur_img, gpu_prev_img, gpu_cur_pts, gpu_reverse_pts, gpu_reverse_status, cv::noArray(), gpu_stream);
            }

            const int valid_count = compactGpuFlow(gpu_cur_pts, gpu_status, gpu_prev_pts,
                                                   gpu_reverse_pts, gpu_reverse_status,
                                                   FLOW_BACK != 0, static_cast<int>(prev_pts.size()),
                                                   cur_pts, gpu_valid_indices);
            if (valid_count >= 0)
            {
                gpu_old_prev_pts = prev_pts;
                gpu_old_ids = ids;
                gpu_old_track_cnt = track_cnt;
                prev_pts.resize(gpu_valid_indices.size());
                ids.resize(gpu_valid_indices.size());
                track_cnt.resize(gpu_valid_indices.size());
                for (size_t i = 0; i < gpu_valid_indices.size(); ++i)
                {
                    const int source_index = gpu_valid_indices[i];
                    prev_pts[i] = gpu_old_prev_pts[source_index];
                    ids[i] = gpu_old_ids[source_index];
                    track_cnt[i] = gpu_old_track_cnt[source_index];
                }
            }
            // printf("gpu temporal optical flow costs: %f ms\n",t_og.toc());
        }
        if (!USE_GPU_ACC_FLOW)
        {
            for (int i = 0; i < int(cur_pts.size()); i++)
                if (status[i] && !inBorder(cur_pts[i]))
                    status[i] = 0;
            reduceVector(prev_pts, status);
            reduceVector(cur_pts, status);
            reduceVector(ids, status);
            reduceVector(track_cnt, status);
        }
        // ROS_DEBUG("temporal optical flow costs: %fms", t_o.toc());
        
        //printf("track cnt %d\n", (int)ids.size());
    }
    if (USE_F_RANSAC)
        rejectWithF();

    if (GPU_PROFILE)
        profile_temporal_ms += temporal_profile_timer.toc();

    for (auto &n : track_cnt)
        n++;

    TicToc feature_profile_timer;
    if (1)
    {
        //rejectWithF();
        ROS_DEBUG("set mask begins");
        TicToc t_m;
        setMask();
        // ROS_DEBUG("set mask costs %fms", t_m.toc());
        // printf("set mask costs %fms\n", t_m.toc());
        ROS_DEBUG("detect feature begins");
        
        int n_max_cnt = MAX_CNT - static_cast<int>(cur_pts.size());
        if(!USE_GPU)
        {
            if (n_max_cnt > 0)
            {
                TicToc t_t;
                if(mask.empty())
                    cout << "mask is empty " << endl;
                if (mask.type() != CV_8UC1)
                    cout << "mask type wrong " << endl;
                cv::goodFeaturesToTrack(cur_img, n_pts, MAX_CNT - cur_pts.size(), 0.01, MIN_DIST, mask);
                // printf("good feature to track costs: %fms\n", t_t.toc());
                std::cout << "n_pts size: "<< n_pts.size()<<std::endl;
            }
            else
                n_pts.clear();
            // sum_n += n_pts.size();
            // printf("total point from non-gpu: %d\n",sum_n);
        }
        
        // ROS_DEBUG("detect feature costs: %fms", t_t.toc());
        // printf("good feature to track costs: %fms\n", t_t.toc());
        else
        {
            if (n_max_cnt > 0)
            {
                if(mask.empty())
                    cout << "mask is empty " << endl;
                if (mask.type() != CV_8UC1)
                    cout << "mask type wrong " << endl;
                TicToc t_g;
                TicToc t_gg;
                if (GPU_PROFILE)
                {
                    TicToc profile_timer;
                    gpu_mask.upload(mask, gpu_stream);
                    profile_upload_ms += profile_timer.toc();
                }
                else
                    gpu_mask.upload(mask, gpu_stream);
                // printf("gpumat cost: %fms\n",t_gg.toc());
                // cout << "new gpu points: "<< MAX_CNT - cur_pts.size()<<endl;
                gpu_detector->detect(gpu_cur_img, gpu_detected_pts, gpu_mask, gpu_stream);
                // std::cout << "d_prevPts size: "<< d_prevPts.size()<<std::endl;
                n_pts.clear();
                if(!gpu_detected_pts.empty())
                {
                    const int detected_count = gpu_detected_pts.rows * gpu_detected_pts.cols;
                    cv::Mat host_detected = host_detected_pts.createMatHeader().rowRange(0, detected_count);
                    if (GPU_PROFILE)
                    {
                        TicToc profile_timer;
                        gpu_detected_pts.download(host_detected, gpu_stream);
                        profile_download_ms += profile_timer.toc();
                        profile_timer.tic();
                        gpu_stream.waitForCompletion();
                        profile_wait_ms += profile_timer.toc();
                    }
                    else
                    {
                        gpu_detected_pts.download(host_detected, gpu_stream);
                        gpu_stream.waitForCompletion();
                    }
                    n_pts.assign(host_detected.ptr<cv::Point2f>(),
                                 host_detected.ptr<cv::Point2f>() + detected_count);
                }
                if (static_cast<int>(n_pts.size()) > n_max_cnt)
                    n_pts.resize(n_max_cnt);
                // sum_n += n_pts.size();
                // printf("total point from gpu: %d\n",sum_n);
                // printf("gpu good feature to track cost: %fms\n", t_g.toc());
            }
            else 
                n_pts.clear();
        }

        ROS_DEBUG("add feature begins");
        TicToc t_a;
        addPoints();
        // ROS_DEBUG("selectFeature costs: %fms", t_a.toc());
        // printf("selectFeature costs: %fms\n", t_a.toc());
    }
    if (GPU_PROFILE)
        profile_feature_ms += feature_profile_timer.toc();

    TicToc undistort_profile_timer;
    cur_un_pts = undistortedPts(cur_pts, m_camera[0]);
    pts_velocity = ptsVelocity(ids, cur_un_pts, cur_un_pts_map, prev_un_pts_map);
    if (GPU_PROFILE)
        profile_undistort_ms += undistort_profile_timer.toc();

    TicToc stereo_profile_timer;
    if(!_img1.empty() && stereo_cam)
    {
        ids_right.clear();
        cur_right_pts.clear();
        cur_un_right_pts.clear();
        right_pts_velocity.clear();
        cur_un_right_pts_map.clear();
        if(!cur_pts.empty())
        {
            //printf("stereo image; track feature on right image\n");
            
            vector<cv::Point2f> reverseLeftPts;
            vector<uchar> status, statusRightLeft;
            if(!USE_GPU_ACC_FLOW)
            {
                TicToc t_check;
                vector<float> err;
                // cur left ---- cur right
                cv::calcOpticalFlowPyrLK(cur_img, rightImg, cur_pts, cur_right_pts, status, err, cv::Size(21, 21), 3);
                // reverse check cur right ---- cur left
                if(FLOW_BACK)
                {
                    cv::calcOpticalFlowPyrLK(rightImg, cur_img, cur_right_pts, reverseLeftPts, statusRightLeft, err, cv::Size(21, 21), 3);
                    for(size_t i = 0; i < status.size(); i++)
                    {
                        if(status[i] && statusRightLeft[i] && inBorder(cur_right_pts[i]) && distance(cur_pts[i], reverseLeftPts[i]) <= 0.5)
                            status[i] = 1;
                        else
                            status[i] = 0;
                    }
                }
                // printf("left right optical flow cost %fms\n",t_check.toc());
            }
            else
            {
                TicToc t_og1;
                // Keep the left points on the device while validating the
                // forward and reverse stereo flow. The compaction kernel
                // returns only valid right points and their source indices.
                uploadPointVector(cur_pts, gpu_prev_pts);
                gpu_flow_level3->calc(gpu_cur_img, gpu_right_img, gpu_prev_pts, gpu_reverse_pts, gpu_status, cv::noArray(), gpu_stream);

                if(FLOW_BACK)
                {
                    // The reverse stereo pass also uses the level-3 flow
                    // object (useInitialFlow=false); the destination is
                    // written by the kernel and needs no initial values.
                    gpu_flow_level3->calc(gpu_right_img, gpu_cur_img, gpu_reverse_pts, gpu_cur_pts, gpu_reverse_status, cv::noArray(), gpu_stream);
                }

                const int valid_count = compactGpuFlow(gpu_reverse_pts, gpu_status, gpu_prev_pts,
                                                       gpu_cur_pts, gpu_reverse_status,
                                                       FLOW_BACK != 0, static_cast<int>(cur_pts.size()),
                                                       cur_right_pts, gpu_valid_indices);
                if (valid_count >= 0)
                {
                    ids_right.resize(gpu_valid_indices.size());
                    for (size_t i = 0; i < gpu_valid_indices.size(); ++i)
                        ids_right[i] = ids[gpu_valid_indices[i]];
                }
                // printf("gpu left right optical flow cost %fms\n",t_og1.toc());
            }
            if (!USE_GPU_ACC_FLOW)
            {
                ids_right = ids;
                reduceVector(cur_right_pts, status);
                reduceVector(ids_right, status);
            }
            // only keep left-right pts
            /*
            reduceVector(cur_pts, status);
            reduceVector(ids, status);
            reduceVector(track_cnt, status);
            reduceVector(cur_un_pts, status);
            reduceVector(pts_velocity, status);
            */
            cur_un_right_pts = undistortedPts(cur_right_pts, m_camera[1]);
            right_pts_velocity = ptsVelocity(ids_right, cur_un_right_pts, cur_un_right_pts_map, prev_un_right_pts_map);
            
        }
        prev_un_right_pts_map = cur_un_right_pts_map;
    }
    if (GPU_PROFILE)
        profile_stereo_ms += stereo_profile_timer.toc();
    if(SHOW_TRACK)
        drawTrack(cur_img, rightImg, ids, cur_pts, cur_right_pts, prevLeftPtsMap);

    prev_img = cur_img;
    if (USE_GPU || USE_GPU_ACC_FLOW)
    {
        // All GPU results consumed by the host above have already been
        // synchronized by their downloads.  The remaining image/flow work
        // is ordered on gpu_stream, so a second full-stream wait here only
        // stalls the CPU and does not change the result.
        gpu_prev_img.swap(gpu_cur_img);
    }
    prev_pts = cur_pts;
    prev_un_pts = cur_un_pts;
    prev_un_pts_map = cur_un_pts_map;
    prev_time = cur_time;
    hasPrediction = false;

    prevLeftPtsMap.clear();
    for(size_t i = 0; i < cur_pts.size(); i++)
        prevLeftPtsMap[ids[i]] = cur_pts[i];

    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
    for (size_t i = 0; i < ids.size(); i++)
    {
        int feature_id = ids[i];
        double x, y ,z;
        x = cur_un_pts[i].x;
        y = cur_un_pts[i].y;
        z = 1;
        double p_u, p_v;
        p_u = cur_pts[i].x;
        p_v = cur_pts[i].y;
        int camera_id = 0;
        double velocity_x, velocity_y;
        velocity_x = pts_velocity[i].x;
        velocity_y = pts_velocity[i].y;

        Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
        xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
        featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
    }

    if (!_img1.empty() && stereo_cam)
    {
        for (size_t i = 0; i < ids_right.size(); i++)
        {
            int feature_id = ids_right[i];
            double x, y ,z;
            x = cur_un_right_pts[i].x;
            y = cur_un_right_pts[i].y;
            z = 1;
            double p_u, p_v;
            p_u = cur_right_pts[i].x;
            p_v = cur_right_pts[i].y;
            int camera_id = 1;
            double velocity_x, velocity_y;
            velocity_x = right_pts_velocity[i].x;
            velocity_y = right_pts_velocity[i].y;

            Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
            xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
            featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
        }
    }

    if (GPU_PROFILE)
    {
        profile_frame_ms += t_r.toc();
        ++profile_frames;
        if (profile_frames % 30 == 0)
        {
            const double scale = 1.0 / 30.0;
            ROS_INFO("[VINS_PROFILE][frontend] frames=%llu avg_ms(frame/temporal/features/undistort/stereo)=%.3f/%.3f/%.3f/%.3f/%.3f upload/download/wait=%.3f/%.3f/%.3f tracked=%zu new=%zu gpu_flow=%d",
                     static_cast<unsigned long long>(profile_frames),
                     profile_frame_ms * scale, profile_temporal_ms * scale,
                     profile_feature_ms * scale, profile_undistort_ms * scale,
                     profile_stereo_ms * scale, profile_upload_ms * scale,
                     profile_download_ms * scale, profile_wait_ms * scale,
                     ids.size(), n_pts.size(), USE_GPU_ACC_FLOW);
            profile_frame_ms = 0.0;
            profile_temporal_ms = 0.0;
            profile_feature_ms = 0.0;
            profile_undistort_ms = 0.0;
            profile_stereo_ms = 0.0;
            profile_upload_ms = 0.0;
            profile_download_ms = 0.0;
            profile_wait_ms = 0.0;
        }
    }
    //printf("feature track whole time %f\n", t_r.toc());
    return featureFrame;
}

void FeatureTracker::rejectWithF()
{
    if (cur_pts.size() >= 8)
    {
        const size_t input_size = cur_pts.size();
        ROS_DEBUG("FM ransac begins");
        TicToc t_f;
        vector<cv::Point2f> un_cur_pts(cur_pts.size()), un_prev_pts(prev_pts.size());
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            Eigen::Vector3d tmp_p;
            m_camera[0]->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + col / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + row / 2.0;
            un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

            m_camera[0]->liftProjective(Eigen::Vector2d(prev_pts[i].x, prev_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + col / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + row / 2.0;
            un_prev_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
        }

        vector<uchar> status;
        cv::findFundamentalMat(un_cur_pts, un_prev_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
        if (status.size() != input_size)
        {
            ROS_DEBUG("FM ransac returned an invalid mask (%zu/%zu); keeping all tracks", status.size(), input_size);
            return;
        }
        const int inlier_count = static_cast<int>(std::count(status.begin(), status.end(), static_cast<uchar>(1)));
        // A homography/fundamental estimate becomes unreliable during severe
        // blur or very small baselines.  In that case retain the optical-flow
        // result instead of deleting nearly every feature.
        const int min_inliers = std::max(20, static_cast<int>(0.5 * input_size));
        if (inlier_count < min_inliers)
        {
            ROS_DEBUG("FM ransac rejected (%d/%zu inliers); keeping all tracks", inlier_count, input_size);
            return;
        }
        int size_a = cur_pts.size();
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(cur_un_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
        ROS_DEBUG("FM ransac: %d -> %lu: %f", size_a, cur_pts.size(), 1.0 * cur_pts.size() / size_a);
        ROS_DEBUG("FM ransac costs: %fms", t_f.toc());
    }
}

void FeatureTracker::readIntrinsicParameter(const vector<string> &calib_file)
{
    std::lock_guard<std::mutex> state_lock(state_mutex);
    if (m_camera.size() == calib_file.size())
        return;
    m_camera.clear();
    for (size_t i = 0; i < calib_file.size(); i++)
    {
        ROS_INFO("reading paramerter of camera %s", calib_file[i].c_str());
        camodocal::CameraPtr camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file[i]);
        m_camera.push_back(camera);
    }
    if (calib_file.size() == 2)
        stereo_cam = 1;
}

void FeatureTracker::showUndistortion(const string &name)
{
    cv::Mat undistortedImg(row + 600, col + 600, CV_8UC1, cv::Scalar(0));
    vector<Eigen::Vector2d> distortedp, undistortedp;
    for (int i = 0; i < col; i++)
        for (int j = 0; j < row; j++)
        {
            Eigen::Vector2d a(i, j);
            Eigen::Vector3d b;
            m_camera[0]->liftProjective(a, b);
            distortedp.push_back(a);
            undistortedp.push_back(Eigen::Vector2d(b.x() / b.z(), b.y() / b.z()));
            //printf("%f,%f->%f,%f,%f\n)\n", a.x(), a.y(), b.x(), b.y(), b.z());
        }
    for (int i = 0; i < int(undistortedp.size()); i++)
    {
        cv::Mat pp(3, 1, CV_32FC1);
        pp.at<float>(0, 0) = undistortedp[i].x() * FOCAL_LENGTH + col / 2;
        pp.at<float>(1, 0) = undistortedp[i].y() * FOCAL_LENGTH + row / 2;
        pp.at<float>(2, 0) = 1.0;
        //cout << trackerData[0].K << endl;
        //printf("%lf %lf\n", p.at<float>(1, 0), p.at<float>(0, 0));
        //printf("%lf %lf\n", pp.at<float>(1, 0), pp.at<float>(0, 0));
        if (pp.at<float>(1, 0) + 300 >= 0 && pp.at<float>(1, 0) + 300 < row + 600 && pp.at<float>(0, 0) + 300 >= 0 && pp.at<float>(0, 0) + 300 < col + 600)
        {
            undistortedImg.at<uchar>(pp.at<float>(1, 0) + 300, pp.at<float>(0, 0) + 300) = cur_img.at<uchar>(distortedp[i].y(), distortedp[i].x());
        }
        else
        {
            //ROS_ERROR("(%f %f) -> (%f %f)", distortedp[i].y, distortedp[i].x, pp.at<float>(1, 0), pp.at<float>(0, 0));
        }
    }
    cv::imshow(name, undistortedImg);
    cv::waitKey(0);
}

vector<cv::Point2f> FeatureTracker::undistortedPts(vector<cv::Point2f> &pts, camodocal::CameraPtr cam)
{
    if ((USE_GPU || USE_GPU_ACC_FLOW) && cam && !pts.empty() &&
        cam->modelType() == camodocal::Camera::PINHOLE &&
        pts.size() <= static_cast<size_t>(MAX_CNT) && !gpu_un_pts.empty())
    {
        vector<double> params;
        cam->writeParameters(params);
        if (params.size() == 8)
        {
            const int point_count = static_cast<int>(pts.size());
            uploadPointVector(pts, gpu_un_pts);
            if (vins_cuda::undistortPinhole(gpu_un_pts, point_count,
                                            params[0], params[1], params[2], params[3],
                                            params[4], params[5], params[6], params[7],
                                            gpu_stream))
            {
                cv::Mat host_points = host_un_pts.createMatHeader().rowRange(0, point_count);
                cv::cuda::GpuMat gpu_un_view = gpu_un_pts.reshape(2, point_count);
                gpu_un_view.download(host_points, gpu_stream);
                gpu_stream.waitForCompletion();

                const cv::Point2f *gpu_points = host_points.ptr<cv::Point2f>();
                bool valid = true;
                for (size_t i = 0; i < pts.size(); ++i)
                {
                    if (!std::isfinite(gpu_points[i].x) || !std::isfinite(gpu_points[i].y))
                    {
                        valid = false;
                        break;
                    }
                }
                if (valid)
                    return vector<cv::Point2f>(gpu_points, gpu_points + pts.size());
            }
            else
                gpu_stream.waitForCompletion();
        }
    }

    vector<cv::Point2f> un_pts;
    un_pts.reserve(pts.size());
    for (unsigned int i = 0; i < pts.size(); i++)
    {
        Eigen::Vector2d a(pts[i].x, pts[i].y);
        Eigen::Vector3d b;
        cam->liftProjective(a, b);
        un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
    }
    return un_pts;
}

vector<cv::Point2f> FeatureTracker::ptsVelocity(vector<int> &ids, vector<cv::Point2f> &pts, 
                                            map<int, cv::Point2f> &cur_id_pts, map<int, cv::Point2f> &prev_id_pts)
{
    vector<cv::Point2f> pts_velocity;
    cur_id_pts.clear();
    for (unsigned int i = 0; i < ids.size(); i++)
    {
        cur_id_pts.insert(make_pair(ids[i], pts[i]));
    }

    // caculate points velocity
    if (!prev_id_pts.empty())
    {
        double dt = cur_time - prev_time;
        
        for (unsigned int i = 0; i < pts.size(); i++)
        {
            std::map<int, cv::Point2f>::iterator it;
            it = prev_id_pts.find(ids[i]);
            if (it != prev_id_pts.end())
            {
                double v_x = (pts[i].x - it->second.x) / dt;
                double v_y = (pts[i].y - it->second.y) / dt;
                pts_velocity.push_back(cv::Point2f(v_x, v_y));
            }
            else
                pts_velocity.push_back(cv::Point2f(0, 0));

        }
    }
    else
    {
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            pts_velocity.push_back(cv::Point2f(0, 0));
        }
    }
    return pts_velocity;
}

void FeatureTracker::drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight, 
                               vector<int> &curLeftIds,
                               vector<cv::Point2f> &curLeftPts, 
                               vector<cv::Point2f> &curRightPts,
                               map<int, cv::Point2f> &prevLeftPtsMap)
{
    //int rows = imLeft.rows;
    int cols = imLeft.cols;
    if (!imRight.empty() && stereo_cam)
        cv::hconcat(imLeft, imRight, imTrack);
    else
        imTrack = imLeft.clone();
    cv::cvtColor(imTrack, imTrack, CV_GRAY2RGB);

    for (size_t j = 0; j < curLeftPts.size(); j++)
    {
        double len = std::min(1.0, 1.0 * track_cnt[j] / 20);
        cv::circle(imTrack, curLeftPts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
    }
    if (!imRight.empty() && stereo_cam)
    {
        for (size_t i = 0; i < curRightPts.size(); i++)
        {
            cv::Point2f rightPt = curRightPts[i];
            rightPt.x += cols;
            cv::circle(imTrack, rightPt, 2, cv::Scalar(0, 255, 0), 2);
            //cv::Point2f leftPt = curLeftPtsTrackRight[i];
            //cv::line(imTrack, leftPt, rightPt, cv::Scalar(0, 255, 0), 1, 8, 0);
        }
    }
    
    map<int, cv::Point2f>::iterator mapIt;
    for (size_t i = 0; i < curLeftIds.size(); i++)
    {
        int id = curLeftIds[i];
        mapIt = prevLeftPtsMap.find(id);
        if(mapIt != prevLeftPtsMap.end())
        {
            cv::arrowedLine(imTrack, curLeftPts[i], mapIt->second, cv::Scalar(0, 255, 0), 1, 8, 0, 0.2);
        }
    }

    //draw prediction
    /*
    for(size_t i = 0; i < predict_pts_debug.size(); i++)
    {
        cv::circle(imTrack, predict_pts_debug[i], 2, cv::Scalar(0, 170, 255), 2);
    }
    */
    //printf("predict pts size %d \n", (int)predict_pts_debug.size());


    //cv::Mat imCur2Compress;
    //cv::resize(imCur2, imCur2Compress, cv::Size(cols, rows / 2));

    cv::imshow("tracking", imTrack);
    cv::waitKey(2);
}


void FeatureTracker::setPrediction(map<int, Eigen::Vector3d> &predictPts)
{
    hasPrediction = true;
    predict_pts.clear();
    predict_pts_debug.clear();
    map<int, Eigen::Vector3d>::iterator itPredict;
    for (size_t i = 0; i < ids.size(); i++)
    {
        //printf("prevLeftId size %d prevLeftPts size %d\n",(int)prevLeftIds.size(), (int)prevLeftPts.size());
        int id = ids[i];
        itPredict = predictPts.find(id);
        if (itPredict != predictPts.end())
        {
            Eigen::Vector2d tmp_uv;
            m_camera[0]->spaceToPlane(itPredict->second, tmp_uv);
            predict_pts.push_back(cv::Point2f(tmp_uv.x(), tmp_uv.y()));
            predict_pts_debug.push_back(cv::Point2f(tmp_uv.x(), tmp_uv.y()));
        }
        else
            predict_pts.push_back(prev_pts[i]);
    }
}


void FeatureTracker::removeOutliers(set<int> &removePtsIds)
{
    std::set<int>::iterator itSet;
    vector<uchar> status;
    for (size_t i = 0; i < ids.size(); i++)
    {
        itSet = removePtsIds.find(ids[i]);
        if(itSet != removePtsIds.end())
            status.push_back(0);
        else
            status.push_back(1);
    }

    reduceVector(prev_pts, status);
    reduceVector(ids, status);
    reduceVector(track_cnt, status);
}


cv::Mat FeatureTracker::getTrackImage()
{
    return imTrack;
}
