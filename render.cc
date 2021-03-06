#include <thread>

#include <raylib.h>
#include <rlgl.h>
#define RAYGUI_IMPLEMENTATION
#include "raygui.h"
#include <gflags/gflags.h>

#include "baller.hpp"
#include "optimization.hpp"

namespace {

  typedef std::map<const Eigen::MatrixXd * const, Texture2D> TextureMap;

  const Sophus::SE3d renderToWorld = Sophus::SE3d::rotZ(M_PI);

  template <int SIZE=256>
  struct ColorMap {
    ColorMap(const std::string input, float min = -100.0, float max = 100.0) : min{min}, max{max} {
      std::ifstream stream(input);
      if (not stream.good()) {
        LOG(FATAL) << "Could not open: [" << input << "]";
      }
      std::string line;
      int i = 0;
      while (std::getline(stream, line)) {
        std::stringstream parser(line);
        float r,g,b;
        parser >> r;
        parser >> g;
        parser >> b;
        data.at(i).at(0) = static_cast<uint8_t>(r * 255.0);
        data.at(i).at(1) = static_cast<uint8_t>(g * 255.0);
        data.at(i).at(2) = static_cast<uint8_t>(b * 255.0);
        ++i;
      }
      stream.close();
    }

    std::array<uint8_t, 3> operator()(float value) const {
      if (value <= min) {
        return data.at(0);
      }
      if (value >= max) {
        return data.back();
      }
      return data.at((value - min)/(max - min) * SIZE);
    }
    const float min;
    const float max;
    std::array<std::array<uint8_t, 3>, SIZE> data;
  };

  void DrawLandmark(const std::array<double, 3> & point, const Color color, float size = 0.15) {
    Eigen::Vector3d pt{point[0], point[1], point[2]};
    const Eigen::Vector3f worldPoint = (renderToWorld.matrix() * pt.homogeneous()).hnormalized().cast<float>();
    DrawCircle3D({worldPoint[0], worldPoint[1], worldPoint[2]}, size, {0, 0, 1}, 0.0, color);
  }

  void DrawLandmark(const double * pointer, const int index, const Color color, float size = 0.15) {
    const std::array<double, 3> point{pointer[3*index + 0],
                                      pointer[3*index + 1],
                                      pointer[3*index + 2]};
    DrawLandmark(point, color, size);
  }

  template <typename NumericType>
    void DrawPose(const Sophus::SE3<NumericType>& worldToCam, Color * const color = nullptr) {
      const Sophus::SE3d renderToCam = renderToWorld * worldToCam.inverse();
      const Eigen::Vector3f origin = renderToCam.translation().template cast<float>();
      const Eigen::Vector3f x = (renderToCam.matrix() * Eigen::Vector3d::UnitX().homogeneous()).hnormalized().template cast<float>();
      if (color) {
        DrawLine3D({origin[0], origin[1], origin[2]}, {x[0], x[1], x[2]}, *color);
      } else {
        DrawLine3D({origin[0], origin[1], origin[2]}, {x[0], x[1], x[2]}, RED);
      }
      const Eigen::Vector3f y = (renderToCam.matrix() * Eigen::Vector3d::UnitY().homogeneous()).hnormalized().template cast<float>();
      if (color) {
        DrawLine3D({origin[0], origin[1], origin[2]}, {y[0], y[1], y[2]}, *color);
      } else {
        DrawLine3D({origin[0], origin[1], origin[2]}, {y[0], y[1], y[2]}, GREEN);
      }
      const Eigen::Vector3f z = (renderToCam.matrix() * Eigen::Vector3d::UnitZ().homogeneous()).hnormalized().template cast<float>();
      if (color) {
        DrawLine3D({origin[0], origin[1], origin[2]}, {z[0], z[1], z[2]}, *color);
      } else {
        DrawLine3D({origin[0], origin[1], origin[2]}, {z[0], z[1], z[2]}, BLUE);
      }
    }

  void DrawPose(const double * pointer, int idx, Color * const color = nullptr) {
    const std::array<double, 6> camera{pointer[baller::CAMERA_SIZE*idx + 0],
                                       pointer[baller::CAMERA_SIZE*idx + 1],
                                       pointer[baller::CAMERA_SIZE*idx + 2],
                                       pointer[baller::CAMERA_SIZE*idx + 3],
                                       pointer[baller::CAMERA_SIZE*idx + 4],
                                       pointer[baller::CAMERA_SIZE*idx + 5]};
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
    DrawPose(Sophus::SE3d{rotation, translation}, color);
  }

  void DrawJacobians(const Eigen::MatrixXd & jacobians,
                     const ColorMap<>& colormap,
                     const std::array<int, 2>& screen,
                     TextureMap & map) {
    constexpr int x_offset = 10;
    constexpr int y_offset = 10;
    if (map.find(&jacobians) != map.end()) {
      const auto texture = map.at(&jacobians);
      DrawTexture(map.at(&jacobians), x_offset, y_offset, WHITE);
      DrawRectangleLines(x_offset - 2, y_offset - 2, texture.width + 4, texture.height + 4, GRAY);
      return;
    }
    Image image;
    image.data = RL_MALLOC(jacobians.rows() * jacobians.cols() * 3);
    image.width = jacobians.cols();
    image.height = jacobians.rows();
    image.mipmaps = 1;
    image.format = PIXELFORMAT_UNCOMPRESSED_R8G8B8;

    uint8_t * ptr = static_cast<uint8_t*>(image.data);
    for (int i = 0; i < jacobians.rows(); ++i) {
      for (int j = 0; j < jacobians.cols(); ++j) {
        auto color = colormap(jacobians(i,j));
        *ptr++ = color.at(0);
        *ptr++ = color.at(1);
        *ptr++ = color.at(2);
      }
    }

    const int width = screen.at(0);
    const int height = screen.at(1);
    const int desired_width = std::floor(width / 3.);
    const int available_height = height - 20;

    if (jacobians.cols() < desired_width) {
      const float ratio = jacobians.rows() / static_cast<float>(jacobians.cols());
      const int new_height = ratio * desired_width;
      if (new_height > available_height) {
        const int new_width = available_height / ratio;
        ImageResizeNN(&image, new_width, available_height);
      } else {
        ImageResizeNN(&image, desired_width, new_height );
      }
    }
    map[&jacobians] = LoadTextureFromImage(image);
    const auto texture = map.at(&jacobians);
    DrawTexture(texture, x_offset, y_offset, WHITE);
    DrawRectangleLines(x_offset - 2, y_offset - 2, texture.width + 4, texture.height + 4, GRAY);
  }

  struct Highlight {
    Highlight(double position[3]) : position{position} {
    }

    Eigen::Vector3f centre() const {
      Eigen::Vector3d cameraPoint{position[0], position[1], position[2]};
      return (renderToWorld.matrix() * cameraPoint.homogeneous()).hnormalized().cast<float>();
    }

    void draw() const {
      const Eigen::Vector3f worldPoint = centre();
      const Vector3 render{worldPoint[0], worldPoint[1], worldPoint[2]};
      const auto color = active ? RED : BLUE;
      DrawCubeWires(render, 1.0f, 1.0f, 1.0f, color);
    }

    BoundingBox box() const {
      const Eigen::Vector3f worldPoint = centre();
      return
        (BoundingBox){Vector3{worldPoint[0] - 1.0/2, worldPoint[1] - 1.0/2, worldPoint[2] - 1.0/2 },
                      Vector3{worldPoint[0] + 1.0/2, worldPoint[1] + 1.0/2, worldPoint[2] + 1.0/2 }};
    }

    void activate() {
      active  = true;
    }

    bool active{false};

    const double * const position;
  };

  struct Highlighter {
    Highlighter(double camera[6], double landmark[3], ceres::Problem& problem, const std::deque<Eigen::MatrixXd> & jacobians)
      : camera{camera},
        landmark{landmark},
        problem{problem},
        jacobians{jacobians} {
          bool has_landmark = problem.HasParameterBlock(landmark);
          bool has_camera = problem.HasParameterBlock(camera);
          if (has_landmark) {
            blocks.push_back({landmark, landmark});
          }
          if (has_camera) {
            blocks.push_back({camera, camera});
          }
          if (has_camera and has_landmark) {
            blocks.push_back({camera, landmark});
          }
        }

    void draw(int index) {
      ceres::Covariance::Options options;
      ceres::Covariance covariance(options);
      const Eigen::MatrixXd jacobian = jacobians.at(index);
      // This is expensive to do, but Ceres annoyingly logs errors whenever
      // covariance.compute fails.
      Eigen::MatrixXd JtJ = jacobian.transpose() * jacobian;
      Eigen::JacobiSVD<Eigen::MatrixXd> svd(JtJ, Eigen::ComputeFullU | Eigen::ComputeFullV);
      if (JtJ.cols() > svd.rank()) {
        //LOG(INFO) << svd.rank() << " : " << JtJ.cols();
        Eigen::MatrixXd singular = svd.singularValues();
        Eigen::MatrixXd S_inverse = singular.asDiagonal().inverse();
        Eigen::MatrixXd inverse = svd.matrixV() * S_inverse * svd.matrixU().transpose();
        //const auto & parameterMap = problem.parameter_map();
        //LOG(INFO) << problem.ParameterBlockLocalSize(landmark);
      } else {
        CHECK(covariance.Compute(blocks, &problem)) ;
        Eigen::Matrix3d block;
        if (landmark) {
          covariance.GetCovarianceBlock(landmark, landmark, block.data());
        }
      }
    }

    const double * const camera;
    const double * const landmark;
    ceres::Problem& problem;
    std::vector<std::pair<const double*, const double*> > blocks;
    const std::deque<Eigen::MatrixXd> & jacobians;
  };

  class ParameterHistoryCallback : public ceres::IterationCallback {
    public:
      ParameterHistoryCallback(baller::BundleAdjustmentInterface* interface) : m_interface{interface} {
        log();
      }

      void log() {
        std::vector<double> cameras{m_interface->cameras(), m_interface->cameras() + m_interface->num_cameras()*baller::CAMERA_SIZE};
        std::vector<double> points{m_interface->points(), m_interface->points() + m_interface->num_points()*baller::POINT_SIZE};
        m_parameter_history.emplace_back(std::make_pair(std::move(cameras), std::move(points)));
      }

      virtual ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary) {
        log();
        return ceres::SOLVER_CONTINUE;
      }

      auto&& history() const {
        return m_parameter_history;
      }

    private:
      baller::BundleAdjustmentInterface * m_interface{nullptr};
      std::vector<std::pair<std::vector<double>, std::vector<double>>> m_parameter_history;
  };
}

DEFINE_string(input, "", "Adjustment input (follows BAL format)");
DEFINE_string(mode, "", "Problem type: slam|localization|mapping");
DEFINE_bool(record, false, "Record screenshots");
std::map<std::string, baller::Mode> kLookup{{"localization", baller::Mode::LOCALIZATION},
                                            {"mapping", baller::Mode::MAPPING},
                                            {"slam", baller::Mode::SLAM}};
std::set<std::string> screenshots;

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  CHECK(not FLAGS_input.empty()) << "Require full path to input problem.txt";
  CHECK(not FLAGS_mode.empty()) << "Require mode type";
  baller::BundleAdjustmentInterface * truth = new baller::Reader{FLAGS_input};
  const auto mode = kLookup.at(FLAGS_mode);
  baller::BundleAdjustmentInterface * observed = new baller::NoisyObserver{*truth, mode};

  auto [problem, options] = build(observed, mode);
  ParameterHistoryCallback callback{observed};
  baller::solve(problem, options, &callback);

  std::deque<Eigen::MatrixXd> jacobians;
  std::deque<double> costs;
  auto && history = callback.history();
  CHECK_GT(history.size(), 0);

  // Cache the jacobian history
  for (auto [cameras, points] : history) {
    observed->set_cameras(cameras);
    observed->set_points(points);
    double cost{0.0};
    ceres::CRSMatrix sparse;
    problem.Evaluate(ceres::Problem::EvaluateOptions(),
                     &cost,
                     nullptr,
                     nullptr,
                     &sparse);
    costs.push_back(cost);
    Eigen::MatrixXd dense;
    baller::CRSToDenseMatrix(sparse, dense);
    jacobians.emplace_back(std::move(dense));
  }
  CHECK_EQ(jacobians.size(), history.size());

  const int width = 3* 800;
  const int height = 2 * 450;
  InitWindow(width, height, "SfM");

  Camera3D camera;
  camera.position = (Vector3){ 16.0f, 18.0f, 0.0f }; // Camera position
  camera.target = (Vector3){ 0.0f, 0.0f, 0.0f };      // Camera looking at point
  camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };          // Camera up vector (rotation towards target)
  camera.fovy = 45.0f;                                // Camera field-of-view Y
  camera.type = CAMERA_PERSPECTIVE;                   // Camera mode type
  SetCameraMode(camera, CAMERA_FREE);
  SetTargetFPS(60);

  ColorMap colormap{"colormap.txt", -20, 20};
  TextureMap jacobian_textures;

  auto start = std::chrono::high_resolution_clock::now();

  Ray ray = { 0 };

  std::vector<Highlight> highlights;
  highlights.push_back({observed->points()});
  Highlighter highlighter{observed->cameras(), observed->points(), problem, jacobians};

  bool animate{true};
  while (!WindowShouldClose()) {
    UpdateCamera(&camera);

    if (IsKeyDown('Z')) {
      camera.target = (Vector3){ 0.0f, 0.0f, 0.0f };
    }

    static std::size_t count = 0;
    static std::size_t index = 0;

    if (animate and ++count > 20) {
      count = 0;
      ++index;
    }
    if (index >= history.size()) {
      index = 0;
    }

    // Set the dataset structure to the camera parameters/point parameters at
    // this history index
    observed->set_cameras(history.at(index).first);
    observed->set_points(history.at(index).second);

    BeginDrawing();
    ClearBackground(RAYWHITE);

    // 3d drawing
    BeginMode3D(camera);

      DrawGrid(10, 1.0f);

      // Draw actual-structure
      for (std::size_t i = 0; i < truth->num_points(); ++i ) {
        DrawLandmark(truth->points(), i, GREEN, 0.5);
      }
      // Draw observed structure
      for (std::size_t i = 0; i < observed->num_points(); ++i ) {
        if (problem.HasParameterBlock(observed->points() + (i * baller::POINT_SIZE))) {
          DrawLandmark(observed->points(), i, RED, 0.25);
        } else {
          DrawLandmark(observed->points(), i, GRAY, 0.25);
        }
      }
      // Draw cameras
      for (std::size_t i = 0; i < observed->num_cameras(); ++i) {
        DrawPose(observed->cameras(), i);
      }
      // Draw highlighted landmarks/cameras
      for (auto && highlight : highlights) {
        if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
          ray = GetMouseRay(GetMousePosition(), camera);
          if (CheckCollisionRayBox(ray, highlight.box())) {
            highlight.activate();
          }
        }
        highlight.draw();
      }

    EndMode3D();

    // 2d drawing
    //highlighter.draw(index);

    // Draw Jacobians
    DrawJacobians(jacobians.at(index),
                  colormap,
                  {width, height},
                  jacobian_textures);

    // Draw observations
    const auto observations = project(truth);
    const auto estimations = project(observed);
    const int spacing  = 10;
    const int startX = width - (estimations.size() * (200 + spacing));
    const int startY = height - 200 - spacing;
    for (const auto [camIdx, mapping] : estimations) {
      const int offsetX = startX + (camIdx * (200 + spacing));
      const int offsetY = startY;
      DrawRectangleLines(offsetX, offsetY, 200, 200, GRAY);
      for (const auto [imgIdx, observation] : mapping) {
        DrawCircle(offsetX + 200/2.0 + observation[0],
            offsetY + 200/2.0 + observation[1],
            2.0, RED);
      }
      for (const auto [imgIdx, observation] : observations.at(camIdx)) {
        DrawCircle(offsetX + baller::CAMERA_WIDTH/2.0 + observation[0],
            offsetY + baller::CAMERA_HEIGHT/2.0 + observation[1],
            1.0, GREEN);
      }
    }

    // Draw text descriptions
    const int margin = 10;
    const int left_column = width - 320 - margin;
    int height = 0;
    DrawRectangle(left_column, 10, 320, 133, Fade(SKYBLUE, 0.5f));
    DrawRectangleLines(left_column, 10, 320, 133, BLUE);
    DrawText("Free camera default controls:", width - 310 , 20, 10, BLACK);
    DrawText("- Mouse Wheel to Zoom in-out", width - 310, 40, 10, DARKGRAY);
    DrawText("- Mouse Wheel Pressed to Pan", width - 310, 60, 10, DARKGRAY);
    DrawText("- Alt + Mouse Wheel Pressed to Rotate", width - 310, 80, 10, DARKGRAY);
    DrawText("- Alt + Ctrl + Mouse Wheel Pressed for Smooth Zoom", width - 310, 100, 10, DARKGRAY);
    DrawText("- Z to zoom to (0, 0, 0)", width - 310, 120, 10, DARKGRAY);
    height += 155;

    // Update slider with current iteration
    GuiSliderBar({left_column, height, 120, 20}, "Iteration",
                  nullptr,
                  static_cast<float>(index),
                  0,
                  static_cast<float>(history.size()));
    height += 30;

    // Animation toggle
    animate = GuiToggle({left_column, height, 20, 20}, nullptr, animate);
    DrawText("Animate?", left_column - 50, height + 5, 10, BLACK);

    // - Fin -
    EndDrawing();

    if (FLAGS_record) {
      static int counter{0};
      std::stringstream stream;
      stream << std::fixed << std::setw(3) << std::setfill('0') << index << "_"
             << std::fixed << std::setw(3) << std::setfill('0') << count;
      if (screenshots.find(stream.str()) == screenshots.end()) {
        std::stringstream filename;
        filename << std::fixed << std::setw(3) << std::setfill('0') << counter++;
        TakeScreenshot((filename.str() + ".png").c_str());
        screenshots.insert(stream.str());
      }
    }
  }
  CloseWindow();
}
