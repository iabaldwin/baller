#include <thread>

#include <raylib.h>
#include <rlgl.h>
#include <gflags/gflags.h>

#include "baller.hpp"
#include "optimization.hpp"


namespace {

  const Sophus::SE3d renderToWorld{};

  void DrawStructurePoint(const std::array<double, 3> & point, const Color color) {
    Eigen::Vector3d pt{point[0], point[1], point[2]};
    const Eigen::Vector3f worldPoint = (renderToWorld.matrix() * pt.homogeneous()).hnormalized().cast<float>();
    DrawPoint3D({worldPoint[0], worldPoint[1], worldPoint[2]}, color);
  }

  void DrawStructurePoint(const double * pointer, const int index, const Color color) {
    const std::array<double, 3> point{pointer[3*index + 0],
      pointer[3*index + 1],
      pointer[3*index + 2]};
    DrawStructurePoint(point, color);
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
    const std::array<double, 6> camera{pointer[9*idx + 0],
      pointer[9*idx + 1],
      pointer[9*idx + 2],
      pointer[9*idx + 3],
      pointer[9*idx + 4],
      pointer[9*idx + 5]};
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

  class VisualizationCallback : public ceres::IterationCallback {
    public:
      VisualizationCallback(ceres::Problem& problem, baller::BundleAdjustmentInterface* estimator, int width, int height) : m_problem{problem},
                                                                                                                            m_width{width},
                                                                                                                            m_height{height},
                                                                                                                            m_estimator{estimator} {
      }

      virtual ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary) {
        // Copy the pose parameter blocks
        std::vector<double> cameras{m_estimator->cameras(), m_estimator->cameras() + m_estimator->num_cameras()*baller::CAMERA_SIZE};
        std::vector<double> points{m_estimator->points(), m_estimator->points() + m_estimator->num_points()*baller::POINT_SIZE};
        m_parameter_history.emplace_back(std::make_pair(std::move(cameras), std::move(points)));
        //double cost{0.0};
        //ceres::CRSMatrix jacobians;
        //m_problem.Evaluate(ceres::Problem::EvaluateOptions(), &cost, nullptr, nullptr, &jacobians);
        //CRSToDenseMatrix(jacobians, m_jacobians);
        //std::lock_guard<std::mutex> lock(m_mutex);
        //if (m_data == nullptr) {
          //m_data = RL_MALLOC(jacobians.num_rows * jacobians.num_cols);
        //}
        //uint8_t * ptr = static_cast<uint8_t*>(m_data);
        //for (int i = 0; i < jacobians.num_rows ; ++i) {
          //for (int j = 0; j < jacobians.num_cols; ++j) {
            //if (m_jacobians(i, j) > 0) {
              //*ptr++ = 255;
            //} else {
              //*ptr++ = 0;
            //}
          //}
        //}
        //m_visualization.width = jacobians.num_cols;
        //m_visualization.height = jacobians.num_rows;
        //m_visualization.mipmaps = 1;
        //m_visualization.format = PIXELFORMAT_UNCOMPRESSED_GRAYSCALE;
        //m_visualization.data = m_data;
        //m_dirty = true;
        return ceres::SOLVER_CONTINUE;
      }

      void draw() {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_dirty) {
          m_output = LoadTextureFromImage(m_visualization);
        }
        //DrawTextureEx(m_output, {0.f, 0.f}, 0, 0.1 ,WHITE);
        DrawTexture(m_output, 0, 0, WHITE);
        m_dirty = false;
      }

      auto history() const {
        return m_parameter_history;
      }

    private:
      void * m_data{nullptr};
      int m_width{-1}, m_height{-1};
      bool m_dirty{false};
      Image m_visualization;
      Texture2D m_output;
      std::mutex m_mutex;
      ceres::Problem& m_problem;
      Eigen::MatrixXd m_jacobians;
      baller::BundleAdjustmentInterface * m_estimator{nullptr};
      std::vector<std::pair<std::vector<double>, std::vector<double>>> m_parameter_history;
  };


}

DEFINE_string(input, "", "Adjustment input (follows BAL format)");

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  CHECK(not FLAGS_input.empty()) << "Require full path to input problem.txt";
  baller::BundleAdjustmentInterface * truth = new baller::Reader{FLAGS_input};
  baller::BundleAdjustmentInterface * observer = new baller::NoisyObserver{*truth};
  baller::BundleAdjustmentInterface * estimator = new baller::Estimator{*observer};
  auto problem = build(observer);

  // Parameters
  //-------------------------------
  const int screenWidth = 3* 800;
  const int screenHeight = 2 * 450;

  VisualizationCallback callback{problem, estimator, screenWidth, screenHeight};
  std::thread solver(&baller::solve, std::ref(problem), &callback);

  InitWindow(screenWidth, screenHeight, "SFM");

  Camera3D camera;
  camera.position = (Vector3){ 10.0f, 10.0f, 10.0f }; // Camera position
  camera.target = (Vector3){ 0.0f, 0.0f, 0.0f };      // Camera looking at point
  camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };          // Camera up vector (rotation towards target)
  camera.fovy = 45.0f;                                // Camera field-of-view Y
  camera.type = CAMERA_PERSPECTIVE;                   // Camera mode type

  Vector3 cubePosition = { 0.0f, 0.0f, 0.0f };
  SetCameraMode(camera, CAMERA_FREE);
  SetTargetFPS(60);

  // Main game loop
  while (!WindowShouldClose()) {
    UpdateCamera(&camera);

    if (IsKeyDown('Z')) camera.target = (Vector3){ 0.0f, 0.0f, 0.0f };

    BeginDrawing();
    ClearBackground(RAYWHITE);

    // Begin 3d visualization
    BeginMode3D(camera);

    // Draw default coordinate axes
    //DrawPose(Sophus::SE3d{});

    //DrawCube(cubePosition, 2.0f, 2.0f, 2.0f, RED);
    //DrawCubeWires(cubePosition, 2.0f, 2.0f, 2.0f, MAROON);
    DrawGrid(10, 1.0f);

    // Draw observed structure
    for (std::size_t i = 0; i < estimator->num_points(); ++i ) {
      DrawStructurePoint(estimator->points(), i, RED);
    }
    // Draw cameras
    for (std::size_t i = 0; i < estimator->num_cameras(); ++i) {
      DrawPose(observer->cameras(), i);
    }
    // Draw actual-structure
    for (std::size_t i = 0; i < truth->num_points(); ++i ) {
      DrawStructurePoint(truth->points(), i, GREEN);
    }

    // Draw optimizer history
    static std::size_t index = 0;
    auto && history = callback.history();
    if (index >= history.size()) {
      index = 0;
    }
    Color color = GRAY;
    for (std::size_t i = 0; i < estimator->num_cameras(); ++i) {
      DrawPose(history.at(index).first.data(), i, &color);
    }
   
    static std::size_t spacer = 0;
    if (++spacer > 100) {
      ++index;
      spacer = 0;
      LOG(INFO) << history.size() << " steps";
    }

    EndMode3D();

    // Begin 2d visualization
    estimator->update();
    // Draw Jacobians
    callback.draw();
    // Draw observations
    const auto projections = estimator->observations_lookup();
    const auto observations = truth->observations_lookup();
    const int spacing  = 10;
    const int startX = screenWidth - (projections.size() * (200 + spacing));
    const int startY = screenHeight - 200 - spacing;
    for (const auto [camIdx, mapping] : projections) {
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

    DrawRectangle(10, 10, 320, 133, Fade(SKYBLUE, 0.5f));
    DrawRectangleLines(10, 10, 320, 133, BLUE);
    DrawText("Free camera default controls:", 20, 20, 10, BLACK);
    DrawText("- Mouse Wheel to Zoom in-out", 40, 40, 10, DARKGRAY);
    DrawText("- Mouse Wheel Pressed to Pan", 40, 60, 10, DARKGRAY);
    DrawText("- Alt + Mouse Wheel Pressed to Rotate", 40, 80, 10, DARKGRAY);
    DrawText("- Alt + Ctrl + Mouse Wheel Pressed for Smooth Zoom", 40, 100, 10, DARKGRAY);
    DrawText("- Z to zoom to (0, 0, 0)", 40, 120, 10, DARKGRAY);
    EndDrawing();
  }
  CloseWindow();
  solver.join();
}
