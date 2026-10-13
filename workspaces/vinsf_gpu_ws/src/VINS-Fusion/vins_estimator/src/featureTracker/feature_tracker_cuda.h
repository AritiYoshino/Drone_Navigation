#pragma once

#include <opencv2/core/cuda.hpp>

namespace vins_cuda
{

// Validate and compact a set of optical-flow results on the CUDA stream. The
// output arrays contain only valid current points and their original indices.
// `use_reverse` enables the forward/backward consistency check.
bool compactFlow(const cv::cuda::GpuMat &current_points,
                 const cv::cuda::GpuMat &forward_status,
                 const cv::cuda::GpuMat &previous_points,
                 const cv::cuda::GpuMat &reverse_points,
                 const cv::cuda::GpuMat &reverse_status,
                 bool use_reverse,
                 int image_width,
                 int image_height,
                 float max_back_error,
                 cv::cuda::GpuMat &compact_points,
                 cv::cuda::GpuMat &compact_indices,
                 cv::cuda::GpuMat &compact_count,
                 cv::cuda::Stream &stream);

// Undistort pinhole points using the same recursive inverse-distortion model
// as camodocal. The operation is in-place on a CV_32FC2 point matrix.
bool undistortPinhole(cv::cuda::GpuMat &points,
                      int point_count,
                      double k1,
                      double k2,
                      double p1,
                      double p2,
                      double fx,
                      double fy,
                      double cx,
                      double cy,
                      cv::cuda::Stream &stream);

}
