#pragma once

#include <array>
#include <iostream>
#include <fstream>
#include <sstream>
#include <filesystem>

#include <Eigen/Core>
#include <glog/logging.h>

#include "definitions.hpp"
#include "optimization.hpp"

namespace baller {

  /**
   * @brief Simple BA interface. Allows for "chaining" of input data, and core
   *        optimization structs.
   */
  class Reader : public BundleAdjustmentInterface {
    public:
      Reader(const std::string& path) : BundleAdjustmentInterface() {
        if (not std::filesystem::is_regular_file(path)) {
          LOG(FATAL) << "[" << path << "] does not exist!";
        }
        std::ifstream input(path);
        if (not input.good()) {
          LOG(FATAL) << "Failed to open: [" + path + "]";
        }

        // Consume header
        std::string line;
        std::getline(input, line);
        std::stringstream stream(line);
        stream >> m_numCameras;
        stream >> m_numPoints;
        stream >> m_numObservations;

        for (int i = 0; i < m_numObservations; ++i) {
          std::getline(input, line);
          std::stringstream stream(line);
          int camIdx, pointIdx;
          stream >> camIdx;
          stream >> pointIdx;
          double x, y;
          stream >> x;
          stream >> y;
          m_observations.push_back(x);
          m_observations.push_back(y);
          m_observations_lookup[camIdx][pointIdx] = {x,y};
        }
        for (int i = 0; i < m_numCameras; ++i) {
          std::array<double, CAMERA_SIZE> camera;
          for (int j = 0; j < CAMERA_SIZE; ++j) {
            std::getline(input, line);
            camera.at(j) = std::atof(line.c_str());
          }
          m_cameras.insert(m_cameras.end(), camera.begin(), camera.end());
        }
        for (int i = 0; i < m_numPoints; ++i) {
          std::array<double, POINT_SIZE> point;
          for (int j = 0; j < POINT_SIZE; ++j) {
            std::getline(input, line);
            point.at(j) = std::atof(line.c_str());
          }
          m_points.insert(m_points.end(), point.begin(), point.end());
        }
        input.close();
      }

      double * observations() override {
        return m_observations.data();
      }

      std::size_t num_observations() const override {
        return m_numObservations;
      }

      double * cameras() override {
        return m_cameras.data();
      }

      void set_cameras(const std::vector<double>& cameras) override {
        CHECK(false) << "Refusing to overwrite gt camera-data";
      }

      void set_points(const std::vector<double>& cameras) override {
        CHECK(false) << "Refusing to overwrite gt point-data";
      }

      std::size_t num_cameras() const override {
        return m_numCameras;
      }

      double* points() override {
        return m_points.data();
      }

      std::size_t num_points() const override {
        return m_numPoints;
      }

      Observations observations_lookup() const override {
        return m_observations_lookup;
      }

    private:
      int m_numCameras{-1}, m_numPoints{-1}, m_numObservations{-1};
      std::vector<double> m_cameras{};
      std::vector<double> m_points{};
      std::vector<double> m_observations{};
      Observations m_observations_lookup{};
  };

  /**
   * @brief Noisy-observer - populates noisy estimates of the input data.
   */
  class NoisyObserver : public BundleAdjustmentInterface {
    public:
      NoisyObserver(BundleAdjustmentInterface & interface, const Mode & mode) : m_interface{interface} {

        // Copy the input camera data
        const auto camera_data = m_interface.cameras();
        std::copy(camera_data,
            camera_data + m_interface.num_cameras() * CAMERA_SIZE,
            std::back_inserter(m_perturbed_cameras));
        // Copy the input point data
        const auto point_data = m_interface.points();
        std::copy(point_data,
            point_data + m_interface.num_points() * POINT_SIZE,
            std::back_inserter(m_perturbed_points));

        if (mode == Mode::SLAM or mode == Mode::LOCALIZATION) {
          // Perturb cameras
          double * perturbed_cameras = m_perturbed_cameras.data();
          for (int i = 0; i < m_perturbed_cameras.size(); i+= CAMERA_SIZE) {
            perturb_camera(perturbed_cameras + i);
          }
        }
        if (mode == Mode::SLAM or mode == Mode::MAPPING) {
          // Perturb points
          double * perturbed_points = m_perturbed_points.data();
          for (int i = 0; i < m_perturbed_points.size(); i+= POINT_SIZE) {
            perturb_point(perturbed_points + i);
          }
        }
      }

      void set_cameras(const std::vector<double>& cameras) override {
        CHECK_EQ(cameras.size(), m_perturbed_cameras.size());
        std::copy(cameras.begin(), cameras.end(), m_perturbed_cameras.begin());
      }

      void set_points(const std::vector<double>& points) override {
        CHECK_EQ(points.size(), m_perturbed_points.size());
        std::copy(points.begin(), points.end(), m_perturbed_points.begin());
      }

      void perturb_point(double * point) {
        point[0] += (rand() % 10 / (10.0));
        point[1] += (rand() % 10 / (10.0));
        point[2] += (rand() % 10 / (10.0));
      }

      void perturb_camera(double * camera) {
        camera[3] = (rand() % 10 / (0.2 * 10.0));
        camera[4] = (rand() % 10 / (0.2 * 10.0));
        camera[5] = (rand() % 10 / (0.2 * 10.0));
        Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
        Eigen::Vector3d rodrigues{camera[0], camera[1], camera[2]};
        const double a = rodrigues.norm();
        if (0 == a) {
          // No rotation
        } else {
          rodrigues /= a;
          rotation = Eigen::Quaterniond {Eigen::AngleAxisd{a, rodrigues}};
        }
        Sophus::SE3d pose{rotation, Eigen::Vector3d{0, 0, 0}};
        pose *= Sophus::SE3d::rotX(rand() % 10 / 20.0);
        pose *= Sophus::SE3d::rotY(rand() % 10 / 20.0);
        pose *= Sophus::SE3d::rotZ(rand() % 10 / 20.0);
        Eigen::AngleAxisd aa{pose.unit_quaternion()};
        rodrigues = aa.axis();
        rodrigues *= aa.angle();
        camera[0] = rodrigues[0];
        camera[1] = rodrigues[1];
        camera[2] = rodrigues[2];
      }

      double * cameras() override {
        return m_perturbed_cameras.data();
      }

      std::size_t num_cameras() const override {
        return m_interface.num_cameras();
      }

      double * points() override {
        return m_perturbed_points.data();
      }

      std::size_t num_points() const override {
        return m_interface.num_points();
      }

      double * observations() override {
        return m_interface.observations();
      }

      std::size_t num_observations() const override {
        return m_interface.num_observations();
      }

      Observations observations_lookup() const override {
        return m_interface.observations_lookup();
      }

    private:
      BundleAdjustmentInterface& m_interface;
      Observations m_perturbed_observations{};
      std::vector<double> m_perturbed_cameras{};
      std::vector<double> m_perturbed_points{};
  };

  /**
   * @brief Project the 3d structure of the interface into the camera view,
   *        using the camera parameters of the interface
   *
   * @param interface
   *
   * @return
   */
  BundleAdjustmentInterface::Observations project(BundleAdjustmentInterface * const interface) {
    std::vector<baller::Point3d> structure;
    double * points = interface->points();
    for (int i = 0; i < interface->num_points() * POINT_SIZE; i+=3) {
      const double x = points[i + 0];
      const double y = points[i + 1];
      const double z = points[i + 2];
      structure.push_back({x, y, z});
    }

    BundleAdjustmentInterface::Observations observations;
    const double * cameras = interface->cameras();
    for (int i = 0; i < interface->num_cameras() ; ++i) {
      std::vector<baller::Point2d> projections;
      PositionAndRotationProjectionError projector{0, 0, CAMERA_FX};
      for (auto && s : structure) {
        double point[3];
        point[0] = s.at(0);
        point[1] = s.at(1);
        point[2] = s.at(2);
        double projected[2];
        projector.project(cameras + (i * baller::CAMERA_SIZE), point, projected);
        projections.push_back({projected[0], projected[1]});
      }
      for (int j = 0; j < projections.size(); ++j) {
        observations[i][j] = {projections.at(j).at(0),  projections.at(j).at(1)};
      }
    }
    return observations;
  }
}
