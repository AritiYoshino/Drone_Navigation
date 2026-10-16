#include <ceres/ceres.h>
#include <opencv2/core.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudaoptflow.hpp>
#include <opencv2/imgproc.hpp>
#include <cmath>
#include <iostream>

struct Residual {
  template <typename T> bool operator()(const T* x, T* r) const {
    r[0] = x[0] - T(3.0);
    return true;
  }
};

int main() {
  try {
    if (cv::cuda::getCudaEnabledDeviceCount() < 1) return 1;
    cv::cuda::DeviceInfo device;
    if (!device.isCompatible()) return 2;
    cv::Mat image(128, 128, CV_8UC1);
    cv::randu(image, 0, 255);
    cv::cuda::GpuMat frame(image), points, tracked, status;
    auto detector = cv::cuda::createGoodFeaturesToTrackDetector(CV_8UC1, 64);
    detector->detect(frame, points);
    if (points.empty()) return 3;
    auto flow = cv::cuda::SparsePyrLKOpticalFlow::create();
    flow->calc(frame, frame, points, tracked, status);
    cv::Mat downloaded;
    status.download(downloaded);
    const int count = cv::countNonZero(downloaded);
    if (count < 32) return 4;
    double x = 0.0;
    ceres::Problem problem;
    problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<Residual, 1, 1>(new Residual),
        nullptr, &x);
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.dense_linear_algebra_library_type = ceres::CUDA;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << "GPU: " << device.name() << ", sm_"
              << device.majorVersion() << device.minorVersion()
              << "; OpenCV " << CV_VERSION << "; tracked=" << count
              << "; Ceres " << CERES_VERSION_STRING
              << " CUDA DENSE_QR x=" << x << "\n"
              << summary.BriefReport() << std::endl;
    return summary.IsSolutionUsable() && std::abs(x - 3.0) < 1e-5 ? 0 : 5;
  } catch (const std::exception& error) {
    std::cerr << error.what() << std::endl;
    return 6;
  }
}
