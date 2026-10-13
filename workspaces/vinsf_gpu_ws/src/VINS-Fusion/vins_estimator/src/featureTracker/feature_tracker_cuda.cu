#include "feature_tracker_cuda.h"

#include <cuda_runtime.h>
#include <opencv2/core/cuda_stream_accessor.hpp>

namespace
{

__device__ inline float2 loadPoint(const unsigned char *base,
                                   size_t step,
                                   int cols,
                                   int index)
{
    const int row = index / cols;
    const int col = index - row * cols;
    return *reinterpret_cast<const float2 *>(base + static_cast<size_t>(row) * step +
                                             static_cast<size_t>(col) * sizeof(float2));
}

__device__ inline unsigned char loadStatus(const unsigned char *base,
                                           size_t step,
                                           int cols,
                                           int index)
{
    const int row = index / cols;
    const int col = index - row * cols;
    return *(base + static_cast<size_t>(row) * step + static_cast<size_t>(col));
}

__global__ void compactFlowKernel(const unsigned char *current_data,
                                  size_t current_step,
                                  int current_cols,
                                  const unsigned char *forward_status_data,
                                  size_t forward_status_step,
                                  int forward_status_cols,
                                  const unsigned char *previous_data,
                                  size_t previous_step,
                                  int previous_cols,
                                  const unsigned char *reverse_data,
                                  size_t reverse_step,
                                  int reverse_cols,
                                  const unsigned char *reverse_status_data,
                                  size_t reverse_status_step,
                                  int reverse_status_cols,
                                  bool use_reverse,
                                  int count,
                                  int image_width,
                                  int image_height,
                                  float max_back_error_sq,
                                  unsigned char *compact_data,
                                  size_t compact_step,
                                  int *compact_indices,
                                  int *compact_count)
{
    const int index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= count)
        return;

    const unsigned char forward_ok = loadStatus(forward_status_data, forward_status_step,
                                                forward_status_cols, index);
    if (!forward_ok)
        return;

    const float2 current = loadPoint(current_data, current_step, current_cols, index);
    const bool in_border = current.x >= 1.0f && current.x < static_cast<float>(image_width - 1) &&
                           current.y >= 1.0f && current.y < static_cast<float>(image_height - 1);
    if (!in_border)
        return;

    if (use_reverse)
    {
        const unsigned char reverse_ok = loadStatus(reverse_status_data, reverse_status_step,
                                                    reverse_status_cols, index);
        if (!reverse_ok)
            return;

        const float2 previous = loadPoint(previous_data, previous_step, previous_cols, index);
        const float2 reverse = loadPoint(reverse_data, reverse_step, reverse_cols, index);
        const float dx = previous.x - reverse.x;
        const float dy = previous.y - reverse.y;
        if (dx * dx + dy * dy > max_back_error_sq)
            return;
    }

    const int output_index = atomicAdd(compact_count, 1);
    *reinterpret_cast<float2 *>(compact_data + static_cast<size_t>(output_index) * compact_step) = current;
    compact_indices[output_index] = index;
}

__global__ void undistortPinholeKernel(unsigned char *data,
                                       size_t step,
                                       int cols,
                                       int count,
                                       double k1,
                                       double k2,
                                       double p1,
                                       double p2,
                                       double fx,
                                       double fy,
                                       double cx,
                                       double cy)
{
    const int index = blockIdx.x * blockDim.x + threadIdx.x;
    if (index >= count)
        return;

    const float2 input = loadPoint(data, step, cols, index);
    const double mx_d = input.x / fx - cx / fx;
    const double my_d = input.y / fy - cy / fy;
    double mx_u = mx_d;
    double my_u = my_d;

    if (!(k1 == 0.0 && k2 == 0.0 && p1 == 0.0 && p2 == 0.0))
    {
        // Keep the same eight-step recursive inverse-distortion calculation
        // used by camodocal's PinholeCamera::liftProjective().
        for (int i = 0; i < 8; ++i)
        {
            const double mx2 = mx_u * mx_u;
            const double my2 = my_u * my_u;
            const double mxy = mx_u * my_u;
            const double rho2 = mx2 + my2;
            const double rho4 = rho2 * rho2;
            const double radial = k1 * rho2 + k2 * rho4;
            const double dx = mx_u * radial + p2 * (rho2 + 2.0 * mx2) + 2.0 * p1 * mxy;
            const double dy = my_u * radial + p1 * (rho2 + 2.0 * my2) + 2.0 * p2 * mxy;
            mx_u = mx_d - dx;
            my_u = my_d - dy;
        }
    }

    const int row = index / cols;
    const int col = index - row * cols;
    *reinterpret_cast<float2 *>(data + static_cast<size_t>(row) * step +
                                static_cast<size_t>(col) * sizeof(float2)) =
        make_float2(static_cast<float>(mx_u), static_cast<float>(my_u));
}

}

namespace vins_cuda
{

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
                 cv::cuda::Stream &stream)
{
    const int count = current_points.rows * current_points.cols;
    if (count <= 0 || current_points.type() != CV_32FC2 || current_points.cols <= 0 ||
        forward_status.type() != CV_8UC1 || forward_status.cols <= 0 ||
        forward_status.rows * forward_status.cols < count ||
        previous_points.type() != CV_32FC2 || previous_points.cols <= 0 ||
        previous_points.rows * previous_points.cols < count ||
        (use_reverse && (reverse_points.type() != CV_32FC2 || reverse_points.cols <= 0 ||
                         reverse_points.rows * reverse_points.cols < count ||
                         reverse_status.type() != CV_8UC1 || reverse_status.cols <= 0 ||
                         reverse_status.rows * reverse_status.cols < count)) ||
        compact_points.rows < count || compact_points.cols != 1 || compact_points.type() != CV_32FC2 ||
        compact_indices.rows < count || compact_indices.cols != 1 || compact_indices.type() != CV_32SC1 ||
        compact_count.rows != 1 || compact_count.cols != 1 || compact_count.type() != CV_32SC1)
        return false;

    cudaStream_t cuda_stream = cv::cuda::StreamAccessor::getStream(stream);
    cudaError_t error = cudaMemsetAsync(compact_count.data, 0, sizeof(int), cuda_stream);
    if (error != cudaSuccess)
        return false;

    const int threads = 128;
    const int blocks = (count + threads - 1) / threads;
    compactFlowKernel<<<blocks, threads, 0, cuda_stream>>>(
        current_points.data,
        current_points.step,
        current_points.cols,
        forward_status.data,
        forward_status.step,
        forward_status.cols,
        previous_points.data,
        previous_points.step,
        previous_points.cols,
        use_reverse ? reverse_points.data : nullptr,
        use_reverse ? reverse_points.step : 0,
        use_reverse ? reverse_points.cols : 1,
        use_reverse ? reverse_status.data : nullptr,
        use_reverse ? reverse_status.step : 0,
        use_reverse ? reverse_status.cols : 1,
        use_reverse,
        count,
        image_width,
        image_height,
        max_back_error * max_back_error,
        compact_points.data,
        compact_points.step,
        reinterpret_cast<int *>(compact_indices.data),
        reinterpret_cast<int *>(compact_count.data));

    return cudaGetLastError() == cudaSuccess;
}

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
                      cv::cuda::Stream &stream)
{
    if (point_count < 0 || points.empty() || points.type() != CV_32FC2 ||
        points.cols <= 0 || points.rows * points.cols < point_count)
        return false;
    if (point_count == 0)
        return true;

    cudaStream_t cuda_stream = cv::cuda::StreamAccessor::getStream(stream);
    const int threads = 128;
    const int blocks = (point_count + threads - 1) / threads;
    undistortPinholeKernel<<<blocks, threads, 0, cuda_stream>>>(
        points.data,
        points.step,
        points.cols,
        point_count,
        k1,
        k2,
        p1,
        p2,
        fx,
        fy,
        cx,
        cy);
    return cudaGetLastError() == cudaSuccess;
}

}
