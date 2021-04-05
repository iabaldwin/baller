#include <iostream>
#include <fstream>
#include <map>

#include <sophus/se3.hpp>

#include "optimization.hpp"

typedef std::vector<baller::Point3d> Structure;
typedef std::vector<Sophus::SE3d> Views;
typedef std::map<std::size_t, std::map<std::size_t, baller::Point2d>> Observations;

Structure quad() {
  Structure structure;
  structure.push_back({-5, -5, 10});
  structure.push_back({-5, 5, 10});
  structure.push_back({5, 5, 10});
  structure.push_back({5, -5, 10});
  return structure;
}

Structure planar(int /*points*/) {
  Structure structure;
  const float spacing = 2.5;
  for (float x = -5; x < 5; x += spacing) {
    for (float y = -5; y < 5; y += spacing) {
      structure.push_back({x, y, 10});
    }
  }
  return structure;
}

Structure spherical(int points = std::numeric_limits<int>::max()) {
  //See: https://www.cmu.edu/biolphys/deserno/pdf/sphere_equi.pdf
  Structure structure;
  const int N = 400;
  const float range = 2.0;
  const float a = 4 * M_PI * std::pow(range, 2) /  N;
  const float d = std::sqrt(a);
  const float M_theta = std::round(M_PI / d);
  const float d_theta = M_PI / M_theta;
  const float d_phi = a / d_theta;

  for (std::size_t m = 0; m < M_theta-1; ++m) {
    float theta = M_PI * (m + 0.5) / M_theta;
    float M_phi = std::round(2*M_PI * sin(theta) / d_phi);
    for (std::size_t n = 0; n < M_phi -1; ++n) {
      float phi = (2*M_PI*n)/M_phi;
      structure.push_back(
          {range*std::sin(theta)*std::cos(phi),
           range*std::sin(theta)*std::sin(phi),
           10+range*cos(theta)});
      if (structure.size() >= points) {
        return structure;
      }
    }
  }
  return structure;
}

Structure buildSyntheticScene(const std::string& type, int points) {
  if ("planar" == type) {
    return planar(points);
  } else if ("spherical" == type) {
    return spherical(points);
  } else if ("quad" == type) {
    return quad();
  } else {
    LOG(FATAL) << "Unknown type: [" << type << "]";
  }
}

Views buildViews(const Structure& structure, int num_cameras) {
  Views views;
  constexpr float lower = -3.;
  constexpr float upper = 3.;
  const float spacing = (upper - lower) / 10;
  for (float x = lower; x < upper; x+= spacing) {
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
    std::vector<baller::Point2d> projections;
    baller::PositionAndRotationProjectionError projector{0, 0, baller::CAMERA_FX};

    double camera[9];
    baller::to(views.at(i), camera);
    camera[6] = baller::CAMERA_FX;
    camera[7] = 0;
    camera[8] = 0;

    for (auto && s : structure) {
      double point[3];
      point[0] = s[0];
      point[1] = s[1];
      point[2] = s[2];
      double projected[2];
      projector.project(camera, point, projected);
      projections.push_back({projected[0], projected[1]});
    }
    for (std::size_t j = 0; j < projections.size(); ++j) {
      observations[i][j] = projections.at(j);
      ++num_observations;
    }
  }
  return std::make_pair(num_observations, observations);
}

DEFINE_string(output, "", "Output file");
DEFINE_string(structure, "planar", "Structure type planar|spherical");
DEFINE_int32(points, 100, "# points");
DEFINE_int32(cameras, 10, "# cameras");

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  CHECK(not FLAGS_output.empty()) << "Require output file";
  const auto structure = buildSyntheticScene(FLAGS_structure, FLAGS_points);
  const auto views = buildViews(structure, FLAGS_cameras);
  const auto [num_observations, observations] = generateObservations(structure, views);

  std::ofstream stream(FLAGS_output);
  if (not stream.good()) {
    return EXIT_FAILURE;
  }
  stream << views.size() << " " << structure.size() << " " << num_observations << std::endl;

  // Write the output problem in the same format as BAL
  // 1. Observations
  for (auto && o : observations) {
    for (auto && c : o.second) {
      stream << o.first << " " << c.first << "   " << c.second.at(0) << " " << c.second.at(1) << std::endl;
    }
  }
  // 2. Views
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
    stream << baller::CAMERA_DISTORATION_P1 << std::endl;
    stream << baller::CAMERA_DISTORATION_P2 << std::endl;
  }

  // 3. Structure
  for (auto && s : structure) {
    stream << s.at(0) << std::endl;
    stream << s.at(1) << std::endl;
    stream << s.at(2) << std::endl;
  }

  stream.close();
}
