#include <opencv2/core.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudaoptflow.hpp>
#include <ceres/ceres.h>
#include <ceres/version.h>
#include <cmath>
#include <iostream>
#include <stdexcept>

struct Residual {
  template <typename T> bool operator()(const T* x, T* residual) const {
    residual[0] = x[0] - T(3.0);
    return true;
  }
};

int main() {
  try {
    if (cv::cuda::getCudaEnabledDeviceCount() < 1)
      throw std::runtime_error("No CUDA device available");
    cv::cuda::setDevice(0);
    cv::cuda::DeviceInfo device(0);
    std::cout << "GPU: " << device.name() << " compute "
              << device.majorVersion() << '.' << device.minorVersion()
              << "; OpenCV " << CV_VERSION << '\n';
    if (!device.isCompatible()) throw std::runtime_error("OpenCV CUDA architecture mismatch");
    cv::Mat image(240, 320, CV_8UC1);
    cv::RNG(1234).fill(image, cv::RNG::UNIFORM, 0, 255);
    cv::GaussianBlur(image, image, cv::Size(5, 5), 1.0);
    cv::Mat shifted, transform = (cv::Mat_<double>(2, 3) << 1, 0, 2, 0, 1, 0);
    cv::warpAffine(image, shifted, transform, image.size());
    cv::cuda::GpuMat first(image), second(shifted), points, next, status;
    auto corners = cv::cuda::createGoodFeaturesToTrackDetector(CV_8UC1, 200, .01, 8);
    corners->detect(first, points);
    auto flow = cv::cuda::SparsePyrLKOpticalFlow::create(cv::Size(21, 21), 3, 30);
    flow->calc(first, second, points, next, status);
    cv::Mat p, q, valid;
    points.download(p); next.download(q); status.download(valid);
    int count = 0; double error = 0;
    for (int i = 0; i < p.cols; ++i) {
      if (!valid.at<unsigned char>(0, i)) continue;
      const cv::Point2f delta = q.at<cv::Point2f>(0, i) - p.at<cv::Point2f>(0, i);
      error += std::hypot(delta.x - 2, delta.y); ++count;
    }
    const double mean_error = count ? error / count : 1e9;
    std::cout << "CUDA corner detection + optical flow: " << count
              << " points, mean error " << mean_error << " px\n";
    if (count < 20 || mean_error > 2.0)
      throw std::runtime_error("CUDA optical flow numerical check failed");
    double x = 0;
    ceres::Problem problem;
    problem.AddResidualBlock(new ceres::AutoDiffCostFunction<Residual, 1, 1>(new Residual), nullptr, &x);
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.dense_linear_algebra_library_type = ceres::CUDA;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    if (!summary.IsSolutionUsable() || std::abs(x - 3) > 1e-5)
      throw std::runtime_error("Ceres CUDA solve failed: " + summary.FullReport());
    std::cout << "Ceres " << CERES_VERSION_STRING << " CUDA DENSE_QR: x=" << x << "\nPASS\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "FAIL: " << error.what() << '\n'; return 1;
  }
}
