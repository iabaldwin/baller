#pragma once

#include <array>
#include <iostream>
#include <sstream>

#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>
#include <Eigen/Core>
#include <sophus/se3.hpp>

#include "stb_image.h"

namespace slammy {

  class Reader {
    public:

      Reader(const std::string& path) : m_root(path){
      }

    protected:
      const boost::filesystem::path m_root;
  };

  class BALReader : public Reader {

    public:
      BALReader(const std::string& path) : Reader(path) {
        if (not boost::filesystem::is_regular_file(path)) {
          throw std::runtime_error("Expected a single file!");
        }
        std::ifstream input(path);
        if (not input.good()) {
          throw std::runtime_error("Failed to open: [" + path + "]");
        }
   
        // Consume header
        std::string line;
        std::getline(input, line);
        std::cout << line << std::endl;
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
          m_observations[camIdx][pointIdx] = {x,y};
        }
        for (int i = 0; i < m_numCameras; ++i) {
          std::array<double, 9> camera;
          for (int j = 0; j < 9; ++j) {
            std::getline(input, line);
            camera.at(j) = std::atof(line.c_str());
          }
          m_rawCameras.insert(m_rawCameras.end(), camera.begin(), camera.end());
          Eigen::Vector3d rodrigues{camera.at(0), camera.at(1), camera.at(2)};
          const double a = rodrigues.norm();
          Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
          if (0 == a) {
            // No rotation
          } else {
            rodrigues /= a;
            rotation = Eigen::Quaterniond {Eigen::AngleAxisd{a, rodrigues}};
          }
          Eigen::Vector3d translation{camera.at(3), camera.at(4), camera.at(5)};
          m_cameras[i] = Sophus::SE3d{rotation, translation};
        }
        for (int i = 0; i < m_numPoints; ++i) {
          std::array<double, 3> point;
          for (int j = 0; j < 3; ++j) {
            std::getline(input, line);
            point.at(j) = std::atof(line.c_str());
          }
          m_rawPoints.insert(m_rawPoints.end(), point.begin(), point.end());
          m_points[i] = std::move(point);
        }
      }

      auto observations() const {
        return m_observations;
      }

      auto points() const {
        return m_points;
      }

      auto cameras() const {
        return m_cameras;
      }

      double* raw_cameras() {
        return m_rawCameras.data();
      }

      double* raw_points() {
        return m_rawPoints.data();
      }

    private:
      int m_numCameras{-1}, m_numPoints{-1}, m_numObservations{-1};
      std::vector<double> m_rawCameras{};
      std::map<int, Sophus::SE3d> m_cameras{};
      std::vector<double> m_rawPoints{};
      std::map<int, std::array<double, 3>> m_points{};
      std::map<int, std::map<int, std::array<double, 2>>> m_observations{};
  };

  class BirdDataReader : public Reader {
    public:

      BirdDataReader(const std::string & path) : Reader(path) {
        m_calib_directory = m_root / "calib";
        if (not boost::filesystem::is_directory(m_calib_directory)) {
          throw std::runtime_error("[" + m_calib_directory.string() + "] does not exist");
        }
        m_images_directory = m_root / "images";
        if (not boost::filesystem::is_directory(m_images_directory)) {
          throw std::runtime_error("[" + m_images_directory.string() + "] does not exist");
        }
        loadImages();
      }

    private:
      int m_width{-1}, m_height{-1}, m_channels{-1};
      boost::filesystem::path m_calib_directory, m_images_directory;
      std::vector<unsigned char * > m_images;

      void loadImages() {
        for(const auto& entry : boost::make_iterator_range(boost::filesystem::directory_iterator(m_images_directory), {})) {
          unsigned char *img = stbi_load(entry.path().string().c_str(), &m_width, &m_height, &m_channels, 0);
          m_images.push_back(img);
          std::cout << "Loaded: " << entry << std::endl;
        }
      }
  };

}
