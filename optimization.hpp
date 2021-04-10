#pragma once

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include "definitions.hpp"

namespace baller {

struct MappingProjectionError {
  MappingProjectionError(double observed_x,
                         double observed_y,
                         double focal,
                         const std::array<double, 6>& camera)
    : observed_x{observed_x}, observed_y{observed_y}, focal{focal}, camera{camera} {}

  template <typename T>
    void project(const T* const point,
                 T projected[2]) const {
      T c[6];
      c[0] = static_cast<T>(camera.at(0));
      c[1] = static_cast<T>(camera.at(1));
      c[2] = static_cast<T>(camera.at(2));
      c[3] = static_cast<T>(camera.at(3));
      c[4] = static_cast<T>(camera.at(4));
      c[5] = static_cast<T>(camera.at(5));

      T p[3];
      ceres::AngleAxisRotatePoint(c, point, p);
      p[0] += c[3];
      p[1] += c[4];
      p[2] += c[5];

      T xp = p[0] / p[2];
      T yp = p[1] / p[2];

      projected[0] = focal * xp;
      projected[1] = focal * yp;
    }

  template <typename T>
    bool operator()(const T* const point,
                    T* residuals) const {
      T projected[2];
      project(point, projected);
      residuals[0] = projected[0]- observed_x;
      residuals[1] = projected[1] - observed_y;
      return true;
    }

  static ceres::CostFunction* Create(const double observed_x,
                                     const double observed_y,
                                     const double focal,
                                     const std::array<double, 6>& camera) {
    return (new ceres::AutoDiffCostFunction<MappingProjectionError, 2, 3>(
          new MappingProjectionError(observed_x, observed_y, focal, camera)));
  }

  const double focal;
  const double observed_x;
  const double observed_y;
  const std::array<double, 6> camera;
};

struct PriorError {
  PriorError(const std::array<double, 3> & location)
    : location{location} {}

  template <typename T>
    bool operator()(const T* const point,
                    T* residuals) const {
      T l[3];
      l[0] = static_cast<T>(location.at(0));
      l[1] = static_cast<T>(location.at(1));
      l[2] = static_cast<T>(location.at(2));
      residuals[0] = point[0] - l[0];
      residuals[1] = point[1] - l[1];
      residuals[2] = point[2] - l[2];
      return true;
    }

  static ceres::CostFunction* Create(const std::array<double, 3> location) {
    return (new ceres::AutoDiffCostFunction<PriorError, 3, 3>(
          new PriorError(location)));
  }

  const std::array<double, 3> location;
};


struct PositionAndRotationProjectionErrorLocalization {
  PositionAndRotationProjectionErrorLocalization(double observed_x,
                                                 double observed_y,
                                                 double focal,
                                                 const std::array<double, 3>& point)
    : observed_x{observed_x},
      observed_y{observed_y},
      focal{focal},
      point{point} {}

  template <typename T>
    void project(const T* const camera,
                 T projected[2]) const {
      T landmark[3];
      landmark[0] = static_cast<T>(point[0]);
      landmark[1] = static_cast<T>(point[1]);
      landmark[2] = static_cast<T>(point[2]);

      T p[3];
      ceres::AngleAxisRotatePoint(camera, landmark, p);
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
        T* residuals) const {
      T projected[2];
      project(camera, projected);
      residuals[0] = projected[0]- observed_x;
      residuals[1] = projected[1] - observed_y;
      return true;
    }

  static ceres::CostFunction* Create(const double observed_x,
                                     const double observed_y,
                                     const double focal,
                                     const std::array<double, 3>& point) {
    return (new ceres::AutoDiffCostFunction<PositionAndRotationProjectionErrorLocalization, 2, 6>(
          new PositionAndRotationProjectionErrorLocalization(observed_x, observed_y, focal, point)));
  }

  const double focal;
  const double observed_x;
  const double observed_y;
  const std::array<double, 3> point;
};

struct PositionAndRotationProjectionError {
  PositionAndRotationProjectionError(double observed_x, double observed_y, double focal)
    : observed_x{observed_x}, observed_y{observed_y}, focal{focal} {}

  template <typename T>
    void project(const T* const camera,
                 const T* const point,
                 T projected[2]) const {
      T p[3];
      ceres::AngleAxisRotatePoint(camera, point, p);
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
    return (new ceres::AutoDiffCostFunction<PositionAndRotationProjectionError, 2, 6, baller::POINT_SIZE>(
          new PositionAndRotationProjectionError(observed_x, observed_y, focal)));
  }

  const double focal;
  const double observed_x;
  const double observed_y;
};

struct PositionRotationAndFocalLengthProjectionError {
  PositionRotationAndFocalLengthProjectionError(double observed_x, double observed_y)
    : observed_x(observed_x), observed_y(observed_y) {}

  template <typename T>
    void project(const T* const camera,
                 const T* const point,
                 T projected[2]) const {
      T p[3];
      ceres::AngleAxisRotatePoint(camera, point, p);
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
    return (new ceres::AutoDiffCostFunction<PositionRotationAndFocalLengthProjectionError, 2, baller::CAMERA_SIZE, baller::POINT_SIZE>(
          new PositionRotationAndFocalLengthProjectionError(observed_x, observed_y)));
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

std::pair<ceres::Problem, ceres::Solver::Options> buildLocalizationProblem(BundleAdjustmentInterface * ba, int num_cameras = std::numeric_limits<int>::max()) {
  CHECK_NOTNULL(ba);
  ceres::Problem problem;
  double * cameras = ba->cameras();
  double * points = ba->points();
  CHECK_GT(ba->observations_lookup().size(), 0);
  for (auto [camIdx, observations] : ba->observations_lookup()) {
    double * camera = cameras + (camIdx * baller::CAMERA_SIZE);
    for (auto [pointIdx, observation] : observations) {
      double * structure = points + (pointIdx * baller::POINT_SIZE);
      std::array<double, 3> landmark{structure[0], structure[1], structure[2]};
      ceres::CostFunction * f = baller::PositionAndRotationProjectionErrorLocalization::Create(observation.at(0),
                                                                                               observation.at(1),
                                                                                               baller::CAMERA_FX,
                                                                                               landmark);

      problem.AddResidualBlock(f, nullptr, camera);
    }
  }
  ceres::Solver::Options options;
  return std::make_pair(std::move(problem), std::move(options));
}

std::pair<ceres::Problem, ceres::Solver::Options> buildMappingProblem(BundleAdjustmentInterface * ba) {
  CHECK_NOTNULL(ba);
  ceres::Problem problem;
  double * cameras = ba->cameras();
  double * points = ba->points();
  CHECK_GT(ba->observations_lookup().size(), 0);
  for (auto [camIdx, observations] : ba->observations_lookup()) {
    double * camera = cameras + (camIdx * baller::CAMERA_SIZE);
    for (auto [pointIdx, observation] : observations) {
      double * structure = points + (pointIdx * baller::POINT_SIZE);
      std::array<double, 6> tmp{camera[0], camera[1], camera[2], camera[3], camera[4], camera[5]};
      ceres::CostFunction * f = baller::MappingProjectionError::Create(observation.at(0),
                                                                       observation.at(1),
                                                                       baller::CAMERA_FX,
                                                                       tmp);
      problem.AddResidualBlock(f, nullptr, structure);
    }
  }
  ceres::Solver::Options options;
  return std::make_pair(std::move(problem), std::move(options));
}

std::pair<ceres::Problem, ceres::Solver::Options> buildSLAMProblem(BundleAdjustmentInterface * ba) {
  CHECK_NOTNULL(ba);
  ceres::Problem problem;
  double * cameras = ba->cameras();
  double * points = ba->points();
  CHECK_GT(ba->observations_lookup().size(), 0);

  ceres::Solver::Options options;
  options.linear_solver_ordering.reset(new ceres::ParameterBlockOrdering);
  for (auto [camIdx, observations] : ba->observations_lookup()) {
    double * camera = cameras + (camIdx * baller::CAMERA_SIZE);
    for (auto [pointIdx, observation] : observations) {
      ceres::CostFunction * f = baller::PositionAndRotationProjectionError::Create(observation.at(0), observation.at(1), baller::CAMERA_FX);
      double * structure = points + (pointIdx * baller::POINT_SIZE);
      options.linear_solver_ordering->AddElementToGroup(camera, 1);
      options.linear_solver_ordering->AddElementToGroup(structure, 0);
      problem.AddResidualBlock(f,
          nullptr,
          camera,
          structure);
    }
  }
  return std::make_pair(std::move(problem), std::move(options));
}

std::pair<ceres::Problem, ceres::Solver::Options> build(BundleAdjustmentInterface * ba, const Mode & mode) {
  switch (mode) {
    case Mode::LOCALIZATION:
      return buildLocalizationProblem(ba, 1);
    case Mode::MAPPING:
      return buildMappingProblem(ba);
    case Mode::SLAM:
      return buildSLAMProblem(ba);
    default:
      LOG(FATAL) << "Unknown mode!";
  }
}

void solve(ceres::Problem& problem, ceres::Solver::Options options, ceres::IterationCallback * callback = nullptr) {
  options.linear_solver_type = ceres::DENSE_SCHUR;
  // Supress clutter
  options.minimizer_progress_to_stdout = false;
  // Update state on every iteration
  options.update_state_every_iteration = true;
  if (callback != nullptr) {
    options.callbacks.push_back(callback);
  }
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << std::endl;
}

}
