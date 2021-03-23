#include <thread>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include "raylib.h"
#include "rlgl.h"
#include "IO.h"

namespace {

  const Sophus::SE3d renderToWorld = Sophus::SE3d::rotZ(M_PI);

  /**
   * @brief Convert BAL (y-up, x-right, z-backward), to opencv convention
   */
  //const Sophus::SE3d BalToOpenCV = Sophus::SE3d::rotX(-M_PI);
  const Sophus::SE3d BalToOpenCV = Sophus::SE3d{};

  void DrawStructurePoint(const std::array<double, 3> & point) {
    Eigen::Vector3d pt{point[0], point[1], point[2]};
    const Eigen::Vector3f worldPoint = (renderToWorld.matrix() * pt.homogeneous()).hnormalized().cast<float>();
    DrawPoint3D({worldPoint[0], worldPoint[1], worldPoint[2]}, RED);
  }

  void DrawStructurePoint(const double * pointer, const int index) {
    const std::array<double, 3> point{pointer[3*index + 0],
      pointer[3*index + 1],
      pointer[3*index + 2]};
    DrawStructurePoint(point);
  }

  template <typename NumericType>
    void DrawPose(const Sophus::SE3<NumericType>& camToWorld) {
      const Sophus::SE3d renderToCam = renderToWorld * camToWorld.inverse() * BalToOpenCV;
      const Eigen::Vector3f origin = renderToCam.translation().template cast<float>();
      const Eigen::Vector3f x = (renderToCam.matrix() * Eigen::Vector3d::UnitX().homogeneous()).hnormalized().template cast<float>();
      DrawLine3D({origin[0], origin[1], origin[2]}, {x[0], x[1], x[2]}, RED);
      const Eigen::Vector3f y = (renderToCam.matrix() * Eigen::Vector3d::UnitY().homogeneous()).hnormalized().template cast<float>();
      DrawLine3D({origin[0], origin[1], origin[2]}, {y[0], y[1], y[2]}, GREEN);
      const Eigen::Vector3f z = (renderToCam.matrix() * Eigen::Vector3d::UnitZ().homogeneous()).hnormalized().template cast<float>();
      DrawLine3D({origin[0], origin[1], origin[2]}, {z[0], z[1], z[2]}, BLUE);
    }

  void DrawPose(const double * pointer, int idx) {
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
    DrawPose(Sophus::SE3d{rotation, translation});
  }

  struct SnavelyReprojectionError {
    SnavelyReprojectionError(double observed_x, double observed_y)
      : observed_x(observed_x), observed_y(observed_y) {}

    template <typename T>
      bool operator()(const T* const camera,
          const T* const point,
          T* residuals) const {
        // camera[0,1,2] are the angle-axis rotation.
        T p[3];
        ceres::AngleAxisRotatePoint(camera, point, p);

        // camera[3,4,5] are the translation.
        p[0] += camera[3];
        p[1] += camera[4];
        p[2] += camera[5];

        // Compute the center of distortion. The sign change comes from
        // the camera model that Noah Snavely's Bundler assumes, whereby
        // the camera coordinate system has a negative z axis.
        T xp = -p[0] / p[2];
        T yp = -p[1] / p[2];

        // Apply second and fourth order radial distortion.
        const T& l1 = camera[7];
        const T& l2 = camera[8];
        T r2 = xp * xp + yp * yp;
        T distortion = 1.0 + r2 * (l1 + l2 * r2);

        // Compute final projected point position.
        const T& focal = camera[6];
        T predicted_x = focal * distortion * xp;
        T predicted_y = focal * distortion * yp;

        // The error is the difference between the predicted and observed position.
        residuals[0] = predicted_x - observed_x;
        residuals[1] = predicted_y - observed_y;

        return true;
      }

    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction* Create(const double observed_x,
        const double observed_y) {
      return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 9, 3>(
            new SnavelyReprojectionError(observed_x, observed_y)));
    }

    double observed_x;
    double observed_y;
  };

  ceres::Problem build(slammy::BALReader* bal) {
    ceres::Problem problem;
    double * raw_camera_parameters = bal->raw_cameras();
    double * raw_point_parameters = bal->raw_points();
    for (auto [camIdx, observations] : bal->observations()) {
      for (auto [pointIdx, observation] : observations) {
        ceres::CostFunction * f = SnavelyReprojectionError::Create(observation.at(0), observation.at(1));
        double * camera_parameters = raw_camera_parameters + (camIdx * 9);
        double * structure_parameters = raw_point_parameters + (pointIdx * 3);
        problem.AddResidualBlock(f,
            nullptr,
            camera_parameters,
            structure_parameters);
      }
    }
    return problem;
  }

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

  class VisualizationCallback : public ceres::IterationCallback {
    public:
      //static constexpr int maximumWidth = 2000;
      //static constexpr int maximumHeight = 2000;
      VisualizationCallback(ceres::Problem& problem, int width, int height) : m_problem{problem}, m_width{width}, m_height{height} {
      }

      virtual ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary) {
        double cost = 0.0;
        ceres::CRSMatrix jacobians;
        m_problem.Evaluate(ceres::Problem::EvaluateOptions(), &cost, nullptr, nullptr, &jacobians);
        //CRSToDenseMatrix(jacobians, mJacobians);
        mJacobians = Eigen::MatrixXd::Random(jacobians.num_rows, jacobians.num_cols);
        std::lock_guard<std::mutex> lock(m_mutex);
       
        if (m_data == nullptr) {
          m_data = RL_MALLOC(jacobians.num_rows * jacobians.num_cols);
        }
        auto ptr = static_cast<uint8_t*>(m_data);
        for (int i = 0; i < 20000; ++i) {
          for (int j = 0; j < 3000; ++j) {
            if (mJacobians(i, j) > 0) {
              *ptr++ = 255;
            } else {
              *ptr++ = 0;
            }
          }
        }
        m_visualization.width = jacobians.num_cols;
        m_visualization.height = jacobians.num_rows;
        m_visualization.mipmaps = 1;
        m_visualization.format = PIXELFORMAT_UNCOMPRESSED_GRAYSCALE;
        m_visualization.data = m_data;
        m_dirty = true;
        return ceres::SOLVER_CONTINUE;
      }

      void draw() {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_dirty) {
          //ImageResize(&m_visualization, 500, 500);
          m_output = LoadTextureFromImage(m_visualization);
        }
        DrawTextureEx(m_output, {0.f, 0.f}, 0, 0.1 ,WHITE);
        m_dirty = false;
      }

    private:
      // Trying to free a STL pointer
      //std::vector<uint8_t> m_data;
      void * m_data{nullptr};
      int m_width{-1}, m_height{-1};
      bool m_dirty{false};
      Image m_visualization;
      Texture2D m_output;
      std::mutex m_mutex;
      ceres::Problem& m_problem;
      Eigen::MatrixXd mJacobians;
  };

  void run_solver(ceres::Problem& problem, VisualizationCallback* callback = nullptr) {
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    if (callback != nullptr) {
      options.callbacks.push_back(callback);
    }
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << std::endl;
  }
}

int main(int argc, char* argv[]) {
  //slammy::BALReader * reader = new slammy::BALReader{"/Users/ianbaldwin/Downloads/BAL/problem-21-11315-pre.txt"};
  slammy::BALReader * reader = new slammy::BALReader{"/Users/ianbaldwin/code/slammy/build/simple.txt"};

  const auto points = reader->points();
  const auto cameras = reader->cameras();
  auto problem = build(reader);

  // Parameters
  //-------------------------------
  const int screenWidth = 3* 800;
  const int screenHeight = 2 * 450;

  VisualizationCallback callback{problem, screenWidth, screenHeight};
  //std::thread solverThread(&run_solver, std::ref(problem), &callback);

  InitWindow(screenWidth, screenHeight, "SFM illustrator");

  // Define the camera to look into our 3d world
  //Camera3D camera = { 0 };
  Camera3D camera;
  camera.position = (Vector3){ 10.0f, 10.0f, 10.0f }; // Camera position
  camera.target = (Vector3){ 0.0f, 0.0f, 0.0f };      // Camera looking at point
  camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };          // Camera up vector (rotation towards target)
  camera.fovy = 45.0f;                                // Camera field-of-view Y
  camera.type = CAMERA_PERSPECTIVE;                   // Camera mode type

  Vector3 cubePosition = { 0.0f, 0.0f, 0.0f };

  SetCameraMode(camera, CAMERA_FREE); // Set a free camera mode

  SetTargetFPS(60);                   // Set our game to run at 60 frames-per-second
  //--------------------------------------------------------------------------------------

  // Main game loop
  while (!WindowShouldClose())        // Detect window close button or ESC key
  {
    // Update
    //----------------------------------------------------------------------------------
    UpdateCamera(&camera);          // Update camera

    if (IsKeyDown('Z')) camera.target = (Vector3){ 0.0f, 0.0f, 0.0f };
    //----------------------------------------------------------------------------------

    // Draw
    //----------------------------------------------------------------------------------
    BeginDrawing();

    ClearBackground(RAYWHITE);

    BeginMode3D(camera);

    DrawPose(Sophus::SE3d{});

    //DrawCube(cubePosition, 2.0f, 2.0f, 2.0f, RED);
    //DrawCubeWires(cubePosition, 2.0f, 2.0f, 2.0f, MAROON);

    DrawGrid(10, 1.0f);

    // Draw structure
    for (const auto [idx, point] : points) {
      DrawStructurePoint(reader->raw_points(), idx);
    }
    // Draw cameras
    for (const auto [idx, camera] : cameras) {
      DrawPose(reader->raw_cameras(), idx);
    }


    EndMode3D();

    // Draw 3d updates
    callback.draw();
    // Draw observations

    const auto observations = reader->observations();

    const int spacing  = 10;
    const int startX = screenWidth - (observations.size() * (200 + spacing));
    const int startY = screenHeight - 200 - spacing;

    for (const auto [camIdx, mapping] : observations) {
      const int offsetX = startX + (camIdx * (200 + spacing));
      const int offsetY = startY;
      DrawRectangleLines(offsetX, offsetY, 200, 200, RED);
      for (const auto [imgIdx, observation] : mapping) {
        DrawCircle(offsetX + observation[0], 
                   offsetY + observation[1],
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
}
