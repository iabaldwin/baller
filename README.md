# JavaScript 3D BAL File Viewer

This application is a JavaScript-based 3D viewer for Bundle Adjustment Language (BAL) files. It parses BAL files, which typically describe 3D scene geometry (cameras and points) and 2D observations, and visualizes them using Three.js.

## Features

*   Parses BAL files containing camera parameters, 3D point coordinates, and 2D observations.
*   Renders the 3D scene:
    *   3D points are shown as green spheres.
    *   Camera poses are shown as 3D axes (RGB for XYZ).
*   Interactive 3D navigation using mouse (orbit, zoom, pan) via Three.js OrbitControls.
*   Displays 2D observations for each camera in separate 2D canvases.
*   Responsive design that adapts to window resizing.

## How to Run

1.  Ensure you have a BAL file (e.g., `simple.txt` is provided with a sample scene). The application expects this file to be in the root directory.
2.  Open the `index.html` file in a modern web browser that supports ES modules.

## BAL File Format

The expected BAL file format is:
1.  First line: `num_cameras num_points num_observations`
2.  Next `num_observations` lines: `camera_index point_index x_projection y_projection`
3.  Next `num_cameras * 9` lines (for each camera):
    *   Rodrigues vector component x
    *   Rodrigues vector component y
    *   Rodrigues vector component z
    *   Translation vector component x
    *   Translation vector component y
    *   Translation vector component z
    *   Focal length (fx)
    *   Radial distortion parameter k1
    *   Radial distortion parameter k2
4.  Next `num_points * 3` lines (for each point):
    *   Point coordinate X
    *   Point coordinate Y
    *   Point coordinate Z

## Technology

*   [Three.js](https://threejs.org/) for 3D rendering and controls.
*   HTML5 Canvas for 2D visualizations.
*   Plain JavaScript (ES modules).

## Note

This JavaScript application focuses on visualization. The bundle adjustment optimization (which is part of the original C++ application this project might be based on) is **not** implemented in this version.
