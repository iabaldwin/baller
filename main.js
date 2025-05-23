// Main JavaScript file for the 3D viewer application
// Imports: Three.js for 3D rendering and OrbitControls for camera manipulation.
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';

/**
 * Parses a BAL (Bundle Adjustment Language) file.
 * The file format consists of:
 * 1. Header: num_cameras num_points num_observations
 * 2. Observations: camera_index point_index x_projection y_projection (num_observations lines)
 * 3. Camera parameters: Rodrigues_x,y,z, Translation_x,y,z, focal_length, k1, k2 (num_cameras * 9 lines)
 * 4. 3D Points: X, Y, Z (num_points * 3 lines)
 * @param {string} fileURL - The URL of the BAL file to parse.
 * @returns {Promise<object>} A promise that resolves to an object containing parsed BAL data.
 * @throws {Error} If fetching or parsing fails.
 */
async function parseBALFile(fileURL) {
    try {
        const response = await fetch(fileURL);
        if (!response.ok) {
            throw new Error(`HTTP error! status: ${response.status}`);
        }
        const text = await response.text();
        const lines = text.trim().split('\n');
        let lineIndex = 0;

        // Read header
        const [numCamerasStr, numPointsStr, numObservationsStr] = lines[lineIndex++].trim().split(/\s+/);
        const numCameras = parseInt(numCamerasStr);
        const numPoints = parseInt(numPointsStr);
        const numObservations = parseInt(numObservationsStr);

        // Read observations
        const observations = [];
        for (let i = 0; i < numObservations; i++) {
            const [cameraIndexStr, pointIndexStr, xStr, yStr] = lines[lineIndex++].trim().split(/\s+/);
            observations.push({
                cameraIndex: parseInt(cameraIndexStr),
                pointIndex: parseInt(pointIndexStr),
                x: parseFloat(xStr),
                y: parseFloat(yStr)
            });
        }

        // Read cameras
        const cameras = [];
        for (let i = 0; i < numCameras; i++) {
            const rodrigues = [
                parseFloat(lines[lineIndex++].trim()), // Rodrigues vector x
                parseFloat(lines[lineIndex++].trim()), // Rodrigues vector y
                parseFloat(lines[lineIndex++].trim())  // Rodrigues vector z
            ];
            const translation = [
                parseFloat(lines[lineIndex++].trim()), // Translation vector x
                parseFloat(lines[lineIndex++].trim()), // Translation vector y
                parseFloat(lines[lineIndex++].trim())  // Translation vector z
            ];
            const focalLength = parseFloat(lines[lineIndex++].trim()); // Focal length
            const k1 = parseFloat(lines[lineIndex++].trim()); // Radial distortion coefficient 1
            const k2 = parseFloat(lines[lineIndex++].trim()); // Radial distortion coefficient 2
            cameras.push({ rodrigues, translation, focalLength, k1, k2 });
        }

        // Read points
        const points = [];
        for (let i = 0; i < numPoints; i++) {
            const x = parseFloat(lines[lineIndex++].trim()); // Point coordinate X
            const y = parseFloat(lines[lineIndex++].trim()); // Point coordinate Y
            const z = parseFloat(lines[lineIndex++].trim()); // Point coordinate Z
            points.push({ x, y, z });
        }

        return { numCameras, numPoints, numObservations, observations, cameras, points };
    } catch (error) {
        console.error("Error parsing BAL file:", error);
        throw error; // Re-throw to allow further handling by the caller
    }
}

// --- Three.js Scene Globals ---
let scene, camera, renderer, controls;

/**
 * Initializes the Three.js scene, camera, renderer, lights, controls, and event listeners.
 */
function init() {
    // Get the container div for the 3D scene
    const container = document.getElementById('scene-container');
    if (!container) {
        console.error("Scene container 'scene-container' not found!");
        return;
    }

    // Scene: Holds all objects, lights, and cameras
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x222222); // Dark grey background for better contrast

    // Camera: Defines the viewpoint (PerspectiveCamera for realistic depth)
    camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
    camera.position.set(5, 5, 5); // Initial camera position
    camera.lookAt(0, 0, 0); // Look at the center of the scene

    // Renderer: Renders the scene using WebGL
    renderer = new THREE.WebGLRenderer({ antialias: true }); // Antialiasing for smoother edges
    renderer.setSize(window.innerWidth, window.innerHeight);
    container.appendChild(renderer.domElement); // Add renderer's canvas to the DOM

    // OrbitControls: Allows interactive camera manipulation (orbit, zoom, pan)
    controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true; // Smooth camera movement
    controls.dampingFactor = 0.25;

    // Lights: Illuminate the scene
    const ambientLight = new THREE.AmbientLight(0x404040); // Soft ambient light for overall illumination
    scene.add(ambientLight);
    const directionalLight = new THREE.DirectionalLight(0xffffff, 0.7); // Directional light to simulate sunlight
    directionalLight.position.set(10, 10, 10); // Position the light
    scene.add(directionalLight);

    // Grid Helper: Visual aid for the scene floor
    const gridHelper = new THREE.GridHelper(20, 20); // Size 20x20, 20 divisions
    scene.add(gridHelper);

    // Window Resize Handler: Adjusts camera aspect and renderer size on window resize
    window.addEventListener('resize', () => {
        camera.aspect = window.innerWidth / window.innerHeight;
        camera.updateProjectionMatrix();
        renderer.setSize(window.innerWidth, window.innerHeight);
    }, false);
}

/**
 * Render loop: Continuously renders the scene and updates controls.
 */
function animate() {
    requestAnimationFrame(animate); // Request the next frame
    if (controls) { 
        controls.update(); // Update OrbitControls (if damping is enabled)
    }
    if (renderer && scene && camera) {
        renderer.render(scene, camera); // Render the scene
    }
}

/**
 * Draws 3D points in the scene.
 * @param {Array<object>} pointsData - Array of point objects {x, y, z}.
 * @param {THREE.Scene} targetScene - The Three.js scene to add points to.
 */
function drawPoints(pointsData, targetScene) {
    if (!pointsData || !targetScene) return;
    // Geometry and material for points (small green spheres)
    const pointGeometry = new THREE.SphereGeometry(0.05, 16, 16); // Radius 0.05
    const pointMaterial = new THREE.MeshBasicMaterial({ color: 0x00ff00 }); // Green

    pointsData.forEach(p => {
        const pointMesh = new THREE.Mesh(pointGeometry, pointMaterial);
        pointMesh.position.set(p.x, p.y, p.z);
        targetScene.add(pointMesh);
    });
    console.log(`${pointsData.length} 3D points drawn.`);
}

/**
 * Draws camera representations (as axes helpers) in the scene.
 * @param {Array<object>} camerasData - Array of camera objects from BAL parsing.
 * @param {THREE.Scene} targetScene - The Three.js scene to add camera representations to.
 */
function drawCameras(camerasData, targetScene) {
    if (!camerasData || !targetScene) return;

    camerasData.forEach(cam => {
        const axesHelper = new THREE.AxesHelper(0.5); // Represents camera pose (RGB for XYZ axes, size 0.5)
        
        // Set camera position
        axesHelper.position.set(cam.translation[0], cam.translation[1], cam.translation[2]);

        // Convert Rodrigues rotation vector to Three.js Quaternion for orientation
        const rodriguesVec = new THREE.Vector3(cam.rodrigues[0], cam.rodrigues[1], cam.rodrigues[2]);
        const angle = rodriguesVec.length(); // Angle of rotation
        const quaternion = new THREE.Quaternion();

        if (angle > 0) { // Avoid division by zero if angle is zero
            const axis = rodriguesVec.normalize(); // Axis of rotation
            quaternion.setFromAxisAngle(axis, angle);
            axesHelper.setRotationFromQuaternion(quaternion);
        }
        // If angle is 0, it's an identity rotation, no need to apply.

        targetScene.add(axesHelper);
    });
    console.log(`${camerasData.length} camera representations drawn.`);
}

/**
 * Draws 2D observations on separate canvases for each camera.
 * @param {object} balData - The parsed BAL data object.
 * @param {string} containerElementId - The ID of the HTML element to host the 2D canvases.
 */
function draw2DObservations(balData, containerElementId) {
    const container = document.getElementById(containerElementId);
    if (!container) {
        console.error(`Container element with ID "${containerElementId}" not found for 2D observations.`);
        return;
    }
    container.innerHTML = ''; // Clear any previous canvases

    if (!balData || !balData.observations || balData.observations.length === 0) {
        console.log("No 2D observations to draw.");
        return;
    }

    // Group observations by camera index
    const observationsByCamera = {};
    for (const obs of balData.observations) {
        if (!observationsByCamera[obs.cameraIndex]) {
            observationsByCamera[obs.cameraIndex] = [];
        }
        observationsByCamera[obs.cameraIndex].push({ x: obs.x, y: obs.y, pointIndex: obs.pointIndex });
    }

    // Create a canvas for each camera's observations
    for (const cameraIndex in observationsByCamera) {
        const canvas = document.createElement('canvas');
        canvas.width = 150; // Fixed size for each 2D observation view
        canvas.height = 150;
        canvas.className = 'observation-canvas'; // Apply CSS styling

        const ctx = canvas.getContext('2d');

        // Draw border for the canvas
        ctx.strokeRect(0, 0, canvas.width, canvas.height);

        // Draw observed points for the current camera
        const cameraObservations = observationsByCamera[cameraIndex];
        cameraObservations.forEach(obs => {
            // BAL image coordinates: origin (0,0) is often image center, y-axis can be upwards or downwards.
            // Canvas coordinates: origin (0,0) is top-left, y-axis is downwards.
            // This transformation assumes BAL (0,0) is center and y is downwards (like canvas).
            const drawX = canvas.width / 2 + obs.x;
            const drawY = canvas.height / 2 + obs.y; 

            ctx.beginPath();
            ctx.arc(drawX, drawY, 2, 0, 2 * Math.PI); // Draw a small red circle for each observation
            ctx.fillStyle = 'red';
            ctx.fill();
        });

        // Add camera index label to the canvas
        ctx.fillStyle = 'black';
        ctx.font = '10px Arial';
        ctx.fillText(`Cam: ${cameraIndex}`, 5, 10); // Label at top-left

        container.appendChild(canvas); // Add the canvas to the designated container
    }
    console.log(`${Object.keys(observationsByCamera).length} camera observation canvases drawn.`);
}


/**
 * Main execution function: Initializes the scene, loads data, and starts rendering.
 */
async function main() {
    init(); // Initialize Three.js scene, camera, renderer, controls
    if (!scene || !renderer) { // Ensure initialization was successful
        console.error("Three.js initialization failed. Aborting main execution.");
        return;
    }

    animate(); // Start the render loop

    try {
        // Load and parse the BAL file (using simple.txt as default)
        const balData = await parseBALFile("simple.txt"); 
        console.log("Parsed BAL data:", balData);

        // Visualize the parsed data
        if (balData.points) {
            drawPoints(balData.points, scene);
        }
        if (balData.cameras) {
            drawCameras(balData.cameras, scene);
        }
        if (balData) {
            draw2DObservations(balData, "2d-observations-container");
        }

    } catch (e) {
        console.error("Main execution failed due to an error in loading or processing BAL data:", e);
    }
}

// Run the main function after the DOM is fully loaded
if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', main);
} else {
    main(); // DOM is already loaded
}
