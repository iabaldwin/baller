#include <iostream>
#include <fstream>
#include <map>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <sophus/se3.hpp>

#include "optimization.hpp"

typedef std::vector<cv::Point3f> Structure;
typedef std::vector<Sophus::SE3d> Views;
typedef std::map<std::size_t, std::map<std::size_t, cv::Point2f>> Observations;

Structure planar() {
  Structure structure;
  for (float x = -5; x < 5; ++x) {
    for (float y = -5; y < 5; ++y) {
      structure.push_back({x, y, 10});
    }
  }
  return structure;
}

Structure spherical() {
  Structure structure;
  const float range = 2.0;
  for (float theta = -M_PI; theta < M_PI; theta += M_PI/10.0) {
    float x = range * cos(theta);
    float y = range * sin(theta);
    for (float z = 2; z < 10.0; z += 0.1) {
      structure.push_back({x,y,z});
    }
  }
  return structure;
}

Structure buildSyntheticScene() {
  //return planar();
  return spherical();
}

Views buildViews(const Structure& structure) {
  Views views;
  for (float x = -2; x < 2; ++x) {
    Sophus::SE3d pose;
    pose.translation()[0] = x;
    views.push_back(pose);
  }
  return views;
}

std::pair<std::size_t, Observations> generateObservations(const Structure& structure, const Views& views) {

  std::size_t num_observations{0};
  Observations observations;
  for (std::size_t i = 0; i < views.size(); ++i) {
    std::vector<cv::Point2f> projections;
    baller::PositionAndRotationReprojectionEror projector{0, 0, baller::CAMERA_FX};

    double camera[9];
    baller::to(views.at(i), camera);
    camera[6] = baller::CAMERA_FX;
    camera[7] = 0;
    camera[8] = 0;

    for (auto && s : structure) {
      double point[3];
      point[0] = s.x;
      point[1] = s.y;
      point[2] = s.z;
      double projected[2];
      projector.project(camera, point, projected);
      //LOG(INFO) << point[0] << " "
                //<< point[1] << " "
                //<< point[2];
      //LOG(INFO) << projected[0] << " "
                //<< projected[1];

      projections.push_back({static_cast<float>(projected[0]), static_cast<float>(projected[1])});
    }

    for (std::size_t j = 0; j < projections.size(); ++j) {
      // Camera, point, obs
      observations[i][j] = projections.at(j);
      ++num_observations;
    }
  }
  return std::make_pair(num_observations, observations);
}

int main(int argc, char* argv[]) {
  const auto structure = buildSyntheticScene();
  const auto views = buildViews(structure);
  const auto [num_observations, observations] = generateObservations(structure, views);
  
  std::ofstream stream("simple.txt");
  stream << views.size() << " " << structure.size() << " " << num_observations << std::endl;
 
  // Write observations
  for (auto && o : observations) {
    for (auto && c : o.second) {
      stream << o.first << " " << c.first << "   " << c.second.x << " " << c.second.y << std::endl;
    }
  }
  // Write views
  for (auto && view : views) {
    const auto rotation = view.unit_quaternion();
    Eigen::AngleAxisd aa{rotation};
    auto axis = aa.axis();
    axis *= aa.angle();
    stream << axis[0] << std::endl;
    stream << axis[1] << std::endl;
    stream << axis[2] << std::endl;
    const auto translation = view.translation();
    stream << translation[0] << std::endl;
    stream << translation[1] << std::endl;
    stream << translation[2] << std::endl;
    stream << baller::CAMERA_FX  << std::endl;
    stream << 0.0 << std::endl;
    stream << 0.0 << std::endl;
  }

  // Write structure
  for (auto && s : structure) {
    stream << s.x << std::endl;
    stream << s.y << std::endl;
    stream << s.z << std::endl;
  }

  stream.close();
}
