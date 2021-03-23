#include <thread>

#include <raylib.h>
#include <rlgl.h>
#include <gflags/gflags.h>

#include "baller.hpp"
#include "optimization.hpp"

namespace {

  typedef std::map<const Eigen::MatrixXd * const, Texture2D> TextureMap;

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

  const Sophus::SE3d renderToWorld = Sophus::SE3d::rotZ(M_PI);

  void DrawStructurePoint(const std::array<double, 3> & point, const Color color, float size = 0.15) {
    Eigen::Vector3d pt{point[0], point[1], point[2]};
    const Eigen::Vector3f worldPoint = (renderToWorld.matrix() * pt.homogeneous()).hnormalized().cast<float>();
    DrawCircle3D({worldPoint[0], worldPoint[1], worldPoint[2]}, size, {0, 0, 1}, 0.0, color);
  }

  void DrawStructurePoint(const double * pointer, const int index, const Color color, float size = 0.15) {
    const std::array<double, 3> point{pointer[3*index + 0],
                                      pointer[3*index + 1],
                                      pointer[3*index + 2]};
    DrawStructurePoint(point, color, size);
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
    const int desired_width = std::floor(width / 5.);
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

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  CHECK(not FLAGS_input.empty()) << "Require full path to input problem.txt";
  CHECK(not FLAGS_mode.empty()) << "Require mode type";
  baller::BundleAdjustmentInterface * truth = new baller::Reader{FLAGS_input};
  const auto mode = kLookup.at(FLAGS_mode);
  baller::BundleAdjustmentInterface * observed = new baller::NoisyObserver{*truth, mode};

  auto problem = build(observed, mode);
  ParameterHistoryCallback callback{observed};
  baller::solve(problem, &callback);

  std::deque<Eigen::MatrixXd> jacobians;
  auto && history = callback.history();
  CHECK_GT(history.size(), 0);

  for (auto [cameras, points] : history) {
    observed->set_cameras(cameras);
    observed->set_points(points);
    double cost{0.0};
    ceres::CRSMatrix sparse;
    Eigen::MatrixXd dense;
    problem.Evaluate(ceres::Problem::EvaluateOptions(), &cost, nullptr, nullptr, &sparse);
    baller::CRSToDenseMatrix(sparse, dense);
    jacobians.emplace_back(std::move(dense));
  }

  const int width = 3* 800;
  const int height = 2 * 450;
  InitWindow(width, height, "SFM");

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

  while (!WindowShouldClose()) {
    UpdateCamera(&camera);

    bool collision = false;
    if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
      ray = GetMouseRay(GetMousePosition(), camera);
    }

    //LOG_EVERY_N(INFO, 100) << camera.position.x << " "
                           //<< camera.position.y << " "
                           //<< camera.position.z;

    if (IsKeyDown('Z')) {
      camera.target = (Vector3){ 0.0f, 0.0f, 0.0f };
    }

    static std::size_t count = 0;
    static std::size_t index = 0;

    if (++count > 20) {
      count = 0;
      ++index;
    }
    if (index >= history.size()) {
      index = 0;
    }

    observed->set_cameras(history.at(index).first);
    observed->set_points(history.at(index).second);

    BeginDrawing();
    ClearBackground(RAYWHITE);

    // Begin 3d visualization
    BeginMode3D(camera);

    DrawGrid(10, 1.0f);

    // Draw observed structure
    for (std::size_t i = 0; i < observed->num_points(); ++i ) {
      DrawStructurePoint(observed->points(), i, RED, 0.25);
    }
    // Draw actual-structure
    for (std::size_t i = 0; i < truth->num_points(); ++i ) {
      DrawStructurePoint(truth->points(), i, GREEN, 0.5);
    }
    // Draw cameras
    for (std::size_t i = 0; i < observed->num_cameras(); ++i) {
      DrawPose(observed->cameras(), i);
    }

    EndMode3D();

    // Draw Jacobians
    CHECK_EQ(jacobians.size(), history.size());
    const Eigen::MatrixXd jacobian = jacobians.at(index);
    DrawJacobians(jacobians.at(index), colormap, {width, height}, jacobian_textures);

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

    DrawRectangle(width - 10 - 320, 10, 320, 133, Fade(SKYBLUE, 0.5f));
    DrawRectangleLines(width - 10 - 320, 10, 320, 133, BLUE);
    DrawText("Free camera default controls:", width - 310 , 20, 10, BLACK);
    DrawText("- Mouse Wheel to Zoom in-out", width - 310, 40, 10, DARKGRAY);
    DrawText("- Mouse Wheel Pressed to Pan", width - 310, 60, 10, DARKGRAY);
    DrawText("- Alt + Mouse Wheel Pressed to Rotate", width - 310, 80, 10, DARKGRAY);
    DrawText("- Alt + Ctrl + Mouse Wheel Pressed for Smooth Zoom", width - 310, 100, 10, DARKGRAY);
    DrawText("- Z to zoom to (0, 0, 0)", width - 310, 120, 10, DARKGRAY);
    EndDrawing();

    const auto now = std::chrono::high_resolution_clock::now();
    const auto elapsed = now - start;

    if (std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count() > 100 and FLAGS_record) {
      static int shot_counter = 0;
      std::stringstream stream;
      stream << std::setw(3) << std::setfill('0') << shot_counter++;
      TakeScreenshot((stream.str() + ".png").c_str());
      start = now;
    }
  }
  CloseWindow();
}
