#include <iostream>
#include <fstream>
#include <map>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <sophus/se3.hpp>

typedef std::vector<cv::Point3f> Structure;
typedef std::vector<Sophus::SE3d> Views;
// Map from camera->point_id->observation
typedef std::map<std::size_t, std::map<std::size_t, cv::Point2f>> Observations;

Structure buildSyntheticScene() {
  Structure structure;
  for (float x = -5; x < 5; ++x) {
    for (float y = -5; y < 5; ++y) {
      structure.push_back({x, y, 10});
    }
  }
  return structure;
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

  cv::Mat K = cv::Mat::zeros(3, 3, CV_32F);
  K.at<float>(0,0) = 100;
  K.at<float>(1,1) = 100;
  K.at<float>(0,2) = 100;
  K.at<float>(1,2) = 100;
  K.at<float>(2,2) = 1;

  std::size_t num_observations{0};
  Observations observations;
  for (std::size_t i = 0; i < views.size(); ++i) {
    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_32F);
    cv::Mat tvec = cv::Mat::zeros(3, 1, CV_32F);
    const auto view = views.at(i);
    tvec.at<float>(0, 0) = view.translation()[0];
    tvec.at<float>(1, 0) = view.translation()[1];
    tvec.at<float>(2, 0) = view.translation()[2];
    std::vector<cv::Point2f> projections;
    cv::projectPoints(structure, rvec, tvec, K, {}, projections);

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
    stream << 100.0 << std::endl;
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
