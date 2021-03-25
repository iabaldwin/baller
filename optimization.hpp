#pragma once

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include "definitions.hpp"

namespace baller {

struct SnavelyReprojectionError {
  SnavelyReprojectionError(double observed_x, double observed_y)
    : observed_x(observed_x), observed_y(observed_y) {}

  template <typename T>
    bool operator()(const T* const camera,
        const T* const point,
        T* residuals) const {
      // camera[0,1,2] are the angle-axis rotation.
      T p[3];
      ceres::AngleAxisRotatePoint(camera, point, p);

      // camera[3,4,5] are the translation.
      p[0] += camera[3];
      p[1] += camera[4];
      p[2] += camera[5];

      // Compute the center of distortion. The sign change comes from
      // the camera model that Noah Snavely's Bundler assumes, whereby
      // the camera coordinate system has a negative z axis.
      T xp = -p[0] / p[2];
      T yp = -p[1] / p[2];

      // Apply second and fourth order radial distortion.
      const T& l1 = camera[7];
      const T& l2 = camera[8];
      T r2 = xp * xp + yp * yp;
      T distortion = 1.0 + r2 * (l1 + l2 * r2);

      // Compute final projected point position.
      const T& focal = camera[6];
      T predicted_x = focal * distortion * xp;
      T predicted_y = focal * distortion * yp;

      // The error is the difference between the predicted and observed position.
      residuals[0] = predicted_x - observed_x;
      residuals[1] = predicted_y - observed_y;
      return true;
    }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(const double observed_x,
      const double observed_y) {
    return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 9, 3>(
          new SnavelyReprojectionError(observed_x, observed_y)));
  }

  const double observed_x;
  const double observed_y;
};

struct PositionAndRotationReprojectionEror {
  PositionAndRotationReprojectionEror(double observed_x, double observed_y, double focal)
    : observed_x{observed_x}, observed_y{observed_y}, focal{focal} {}

  template <typename T>
    void project(const T* const camera,
                 const T* const point,
                 T projected[2]) const {
      // camera[0,1,2] are the angle-axis rotation.
      T p[3];
      ceres::AngleAxisRotatePoint(camera, point, p);
      // camera[3,4,5] are the translation.
      p[0] += camera[3];
      p[1] += camera[4];
      p[2] += camera[5];

      T xp = p[0] / p[2];
      T yp = p[1] / p[2];

      projected[0] = focal * xp;
      projected[1] = focal * yp;
    }

  template <typename T>
    bool operator()(const T* const camera,
        const T* const point,
        T* residuals) const {
  
      T projected[2];
      project(camera, point, projected);
      residuals[0] = projected[0]- observed_x;
      residuals[1] = projected[1] - observed_y;
      return true;
    }

  static ceres::CostFunction* Create(const double observed_x,
                                     const double observed_y,
                                     const double focal) {
    return (new ceres::AutoDiffCostFunction<PositionAndRotationReprojectionEror, 2, 6, baller::POINT_SIZE>(
          new PositionAndRotationReprojectionEror(observed_x, observed_y, focal)));
  }

  const double focal;
  const double observed_x;
  const double observed_y;
};

struct FullReprojectionError {
  FullReprojectionError(double observed_x, double observed_y)
    : observed_x(observed_x), observed_y(observed_y) {}

  template <typename T>
    void project(const T* const camera,
                 const T* const point,
                 T projected[2]) const {
      // camera[0,1,2] are the angle-axis rotation.
      T p[3];
      ceres::AngleAxisRotatePoint(camera, point, p);
      // camera[3,4,5] are the translation.
      p[0] += camera[3];
      p[1] += camera[4];
      p[2] += camera[5];

      T xp = p[0] / p[2];
      T yp = p[1] / p[2];

      const T& focal = camera[6];
      projected[0] = focal * xp;
      projected[1] = focal * yp;
    }

  template <typename T>
    bool operator()(const T* const camera,
        const T* const point,
        T* residuals) const {
  
      T projected[2];
      project(camera, point, projected);
      residuals[0] = projected[0]- observed_x;
      residuals[1] = projected[1] - observed_y;
      return true;
    }

  static ceres::CostFunction* Create(const double observed_x,
      const double observed_y) {
    return (new ceres::AutoDiffCostFunction<FullReprojectionError, 2, baller::CAMERA_SIZE, baller::POINT_SIZE>(
          new FullReprojectionError(observed_x, observed_y)));
  }

  const double observed_x;
  const double observed_y;
};

void CRSToDenseMatrix(const ceres::CRSMatrix& input, Eigen::MatrixXd& output) {
    output.resize(input.num_rows, input.num_cols);
    output.setZero();
    for (int row = 0; row < input.num_rows; ++row) {
      for (int j = input.rows.at(row); j < input.rows.at(row+1); ++j) {
        const int col = input.cols.at(j);
        output(row, col) = input.values.at(j);
      }
    }
  }

ceres::Problem build(BundleAdjustmentInterface * ba) {
    CHECK_NOTNULL(ba);
    ceres::Problem problem;
    double * cameras = ba->cameras();
    double * points = ba->points();
    CHECK_GT(ba->observations_lookup().size(), 0);
    for (auto [camIdx, observations] : ba->observations_lookup()) {
      double * camera = cameras + (camIdx * baller::CAMERA_SIZE);
      for (auto [pointIdx, observation] : observations) {
        ceres::CostFunction * f = baller::PositionAndRotationReprojectionEror::Create(observation.at(0), observation.at(1), baller::CAMERA_FX);
        double * structure = points + (pointIdx * baller::POINT_SIZE);
        problem.AddResidualBlock(f,
                                 nullptr,
                                 camera,
                                 structure);
      }
    }
    return problem;
  }

void solve(ceres::Problem& problem, ceres::IterationCallback * callback = nullptr) {
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    if (callback != nullptr) {
      options.callbacks.push_back(callback);
    }
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << std::endl;
  }

}
