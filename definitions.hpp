#pragma once

#include <map>

#include <sophus/se3.hpp>

namespace baller {
  constexpr std::size_t POINT_SIZE = 3;
  constexpr std::size_t POSE_SIZE = 6;
  constexpr std::size_t CAMERA_SIZE = 9;
  constexpr std::size_t OBSERVATIONS_SIZE = 2;
  constexpr std::size_t CAMERA_WIDTH = 200;
  constexpr std::size_t CAMERA_HEIGHT = 200;
  constexpr std::size_t CAMERA_FX = 150;
  constexpr std::size_t CAMERA_FY = 150;
  constexpr float CAMERA_DISTORATION_P1 = 0;
  constexpr float CAMERA_DISTORATION_P2 = 0;

  typedef std::array<double, 3> Point3d;
  typedef std::array<double, 2> Point2d;

  enum class Mode {
    LOCALIZATION = 0,
    MAPPING = 1,
    SLAM
  };

  Sophus::SE3d from(const double * camera) {
    Eigen::Vector3d translation{camera[3], camera[4], camera[5]};
    Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
    Eigen::Vector3d rodrigues{camera[0], camera[1], camera[2]};
    const double a = rodrigues.norm();
    if (0 == a) {
      // Identity rotation
    } else {
      rodrigues /= a;
      rotation = Eigen::Quaterniond{Eigen::AngleAxisd{a, rodrigues}};
    }
    return Sophus::SE3d{rotation, translation};
  }

  void to(const Sophus::SE3d& camToWorld , double camera[CAMERA_SIZE]) {
    Eigen::AngleAxisd aa{camToWorld.unit_quaternion()};
    auto axis = aa.axis();
    axis *= aa.angle();
    camera[0] = axis[0];
    camera[1] = axis[1];
    camera[2] = axis[2];
    camera[3] = camToWorld.translation()[0];
    camera[4] = camToWorld.translation()[1];
    camera[5] = camToWorld.translation()[2];
  }

  /**
   * @brief Core BA-interface. Operates over raw pointers to interface with
   * ceres more easily
   */
  class BundleAdjustmentInterface {
    public:
      typedef std::map<int, std::map<int, std::array<double, 2>>> Observations;

      /**
       * @brief Pointer to the raw camera data in the BAL format (r,t,f,d1,d2)
       *
       * @return Pointer to densely-packed data
       */
      virtual double * cameras() = 0;

      /**
       * @brief Set camera parameters from the input tightly-packed data
       *
       * @param parameters
       */
      virtual void set_cameras(const std::vector<double>& parameters) = 0;

      /**
       * @brief Number of cameras represented
       *
       * @return # cameras
       */
      virtual std::size_t num_cameras() const = 0;

      virtual double * points() = 0;
      virtual std::size_t num_points() const = 0;
      virtual void set_points(const std::vector<double>& parameters) = 0;

      virtual double * observations() = 0;
      virtual std::size_t num_observations() const = 0;

      /**
       * @brief Mapping of cameras-point indices
       *
       * @return
       */
      virtual Observations observations_lookup() const = 0;
  };
}
