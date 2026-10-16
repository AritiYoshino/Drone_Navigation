#!/usr/bin/env bash
# Run an actual Ceres CUDA DENSE_QR solve; no ROS or flight hardware is used.
set -euo pipefail
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
PROJECT_ROOT=$(cd -- "$SCRIPT_DIR/.." && pwd)
CERES_PREFIX="$PROJECT_ROOT/runtime/local/ceres"
CHECK_BUILD="$PROJECT_ROOT/runtime/checks/ceres_cuda"
CHECK_BIN="$PROJECT_ROOT/runtime/local/bin/drone_ceres_check"
[[ -f "$CERES_PREFIX/include/ceres/ceres.h" ]] || {
  echo "Build CUDA Ceres first: scripts/build_gpu.sh ceres" >&2; exit 1;
}
mkdir -p "$CHECK_BUILD" "$(dirname -- "$CHECK_BIN")"
cat > "$CHECK_BUILD/check.cc" <<'CPP'
#include <ceres/ceres.h>
#include <ceres/version.h>
#include <glog/logging.h>
#include <cmath>
#include <iostream>

class LinearResidual final : public ceres::SizedCostFunction<4, 2> {
 public:
  bool Evaluate(double const* const* parameters, double* residuals,
                double** jacobians) const override {
    const double a[8] = {1, 0, 0, 1, 1, 1, 2, -1};
    const double b[4] = {2, -3, -1, 7};
    for (int i = 0; i < 4; ++i)
      residuals[i] = a[2*i]*parameters[0][0] +
                     a[2*i+1]*parameters[0][1] - b[i];
    if (jacobians && jacobians[0])
      for (int i = 0; i < 8; ++i) jacobians[0][i] = a[i];
    return true;
  }
};

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  if (!ceres::IsDenseLinearAlgebraLibraryTypeAvailable(ceres::CUDA)) {
    std::cerr << "Ceres CUDA dense solver is unavailable\n";
    return 1;
  }
  double x[2] = {-9, 7};
  ceres::Problem problem;
  problem.AddResidualBlock(new LinearResidual, nullptr, x);
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.dense_linear_algebra_library_type = ceres::CUDA;
  options.num_threads = 1;
  options.max_num_iterations = 20;
  options.function_tolerance = 1e-12;
  options.gradient_tolerance = 1e-12;
  options.parameter_tolerance = 1e-12;
  options.logging_type = ceres::SILENT;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  const bool correct = summary.IsSolutionUsable() &&
      summary.linear_solver_type_used == ceres::DENSE_QR &&
      summary.dense_linear_algebra_library_type == ceres::CUDA &&
      std::abs(x[0] - 2) < 1e-8 && std::abs(x[1] + 3) < 1e-8 &&
      summary.final_cost < 1e-16;
  std::cout << "Ceres " << CERES_VERSION_STRING << " CUDA DENSE_QR: x=["
            << x[0] << "," << x[1] << "], final_cost=" << summary.final_cost
            << ", " << (correct ? "PASS" : "FAIL") << '\n';
  if (!correct) std::cerr << summary.FullReport() << '\n';
  return correct ? 0 : 1;
}
CPP
/usr/bin/g++ -std=c++14 -O2 "$CHECK_BUILD/check.cc" \
  -I"$CERES_PREFIX/include" -I/usr/include/eigen3 \
  -L"$CERES_PREFIX/lib" -Wl,-rpath,"$CERES_PREFIX/lib" \
  -Wl,-rpath,/usr/local/cuda-10.2/lib64 \
  -lceres -lglog -o "$CHECK_BIN"
LD_LIBRARY_PATH="$CERES_PREFIX/lib:/usr/local/cuda-10.2/lib64${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
  "$CHECK_BIN"
