/**
 * 文档内容配置文件
 * ==================
 * 
 * 你只需要修改这个文件来更新文档内容！
 * 
 * 每个文档包含：
 * - tag: 标签名（显示在卡片和弹窗中）
 * - title: 标题
 * - subtitle: 副标题
 * - cardBg: 卡片背景图片URL（留空则使用 cardGradient 渐变色）
 * - cardGradient: 卡片渐变色（当 cardBg 为空时使用）
 *   可选值: 'blue', 'purple', 'green', 'orange'
 * - bg: 弹窗顶部图片URL（留空则不显示图片）
 * - content: HTML内容（使用模板字符串 `` 可以多行）
 * 
 * 图片路径：https://berkeleydrone.github.io/images/docs/图片名.png
 * 
 * 可用图片：
 * - drone_parts.png      (硬件零件)
 * - Drone_frame_image.png (CAD图)
 * - calibrated_images.png (校准图像)
 * - accel_noise.png      (加速度计噪声)
 * - gyro_noise.png       (陀螺仪噪声)
 * - summary_noise.png    (噪声汇总)
 * - pipeline_diagram.png (管道图)
 */

const docData = [
  // ========== 0: Hardware ==========
  {
    tag: 'Hardware',
    title: 'Drone Frame & Components',
    subtitle: 'A2RL Racing Drone Platform',
    cardBg: 'https://berkeleydrone.github.io/images/docs/drone_parts.png',
    cardGradient: '',
    bg: 'https://berkeleydrone.github.io/images/docs/drone_parts.png',
    content: `
      <h3>Overview</h3>
      <p>In this project, the <strong>Jetson Xavier</strong> is used as the main compute unit to run the code stack, including sensor drivers, control, perception, and planning modules. The communication is established between the Jetson Xavier (the brain) and the flight controller (the muscle). The desired motor commands are computed on the Jetson board and the commands are sent to the flight controllers to control the motor speed.</p>
      
      <div style="margin: 24px 0; padding: 20px; background: #F8F9FA; border-radius: 12px; border-left: 4px solid #0668E1;">
        <img src="https://berkeleydrone.github.io/images/docs/drone_parts.png" alt="Drone Parts" style="width: 100%; border-radius: 8px; margin-bottom: 12px;">
        <p style="margin: 0; font-size: 0.875rem; color: #65676B;"><strong>Figure 1:</strong> Parts and sensors used in the project</p>
      </div>
      
      <h3>Frame and Fasteners</h3>
      <table>
        <tr><th>Component</th><th>Type</th></tr>
        <tr><td>Frame</td><td>A2RL X DCL SPEC FRAME</td></tr>
        <tr><td>Battery Strap</td><td>Gemfan Kevlar Strap</td></tr>
        <tr><td>XT30 Connectors</td><td>XT30 Male and Female Pairs</td></tr>
      </table>
      
      <h3>Electronics</h3>
      <table>
        <tr><th>Component</th><th>Type</th></tr>
        <tr><td>Motors</td><td>XNOVA Black Thunder 2207 2100KV</td></tr>
        <tr><td>ESC</td><td>Foxeer Reaper F4 128K 65A BL32 4in1</td></tr>
        <tr><td>Flight Controller</td><td>Foxeer H7 MPU6000 FC</td></tr>
        <tr><td>VTX Antenna</td><td>Foxeer 5.8G Micro Lollipop</td></tr>
        <tr><td>VTX Camera</td><td>HDZERO Nano 90</td></tr>
        <tr><td>VTX</td><td>HDZERO Race 3 VTX</td></tr>
        <tr><td>Receiver</td><td>IRC Ghost Atto</td></tr>
      </table>
      
      <h3>Assembly Elements</h3>
      <table>
        <tr><th>Component</th><th>Type</th></tr>
        <tr><td>Jumpers</td><td>Silicone Jumpers Arduino Female</td></tr>
        <tr><td>Heat Shrink Tube</td><td>30mm Heat Shrink Tube</td></tr>
        <tr><td>XT30 Connectors</td><td>XT30 Male and Female Pairs</td></tr>
      </table>
      
      <h3>RC Controller</h3>
      <table>
        <tr><th>Component</th><th>Type</th></tr>
        <tr><td>Controller</td><td>Radiomaster Zorro 4in1</td></tr>
        <tr><td>Transmitter Module</td><td>Ghost Module</td></tr>
      </table>
      
      <h3>Propellers</h3>
      <table>
        <tr><th>Component</th><th>Type</th></tr>
        <tr><td>Propellers</td><td>MCK 51466 V2 (5 inch)</td></tr>
      </table>
      
      <div style="margin: 24px 0; padding: 20px; background: #F8F9FA; border-radius: 12px; border-left: 4px solid #0668E1;">
        <img src="https://berkeleydrone.github.io/images/docs/Drone_frame_image.png" alt="Drone CAD" style="width: 100%; border-radius: 8px; margin-bottom: 12px;">
        <p style="margin: 0; font-size: 0.875rem; color: #65676B;"><strong>Figure 2:</strong> CAD design of the drone assembly</p>
      </div>
      
      <h3>System Architecture</h3>
      <p>The system follows a hierarchical architecture:</p>
      <ul>
        <li><strong>Compute Layer:</strong> Jetson Xavier running ROS2 nodes for perception, planning, and high-level control</li>
        <li><strong>Control Layer:</strong> Foxeer H7 flight controller running Betaflight firmware for low-level stabilization</li>
        <li><strong>Actuation Layer:</strong> 4x XNOVA motors driven by Foxeer ESC for thrust generation</li>
        <li><strong>Communication:</strong> MSP (MultiWii Serial Protocol) for Jetson-to-FC commands, IRC Ghost for RC telemetry</li>
      </ul>
    `
  },

  // ========== 1: Calibration ==========
  {
    tag: 'Calibration',
    title: 'Sensor Calibration',
    subtitle: 'Camera & IMU Calibration Procedures',
    cardBg: 'https://berkeleydrone.github.io/images/docs/calibrated_images.png',
    cardGradient: '',
    bg: 'https://berkeleydrone.github.io/images/docs/calibrated_images.png',
    content: `
      <h3>Overview</h3>
      <p>Sensor calibration is an essential step to be carried out for any real-world robotics platform to work properly and effectively. This is particularly true for <strong>custom-built</strong> platforms, such as the drone used in this project.</p>
      <p>In this project, different calibration packages were used to calibrate the onboard camera, IMU, and motors.</p>
      
      <h3>IMU Calibration</h3>
      <p>For the IMU calibration, we recorded an 11-hour static ROS 2 bag to estimate intrinsic noise characteristics and long-term drift. The bag was converted to ROS 1 format using the <code>rosbags</code> package to enable offline calibration with the Allan Variance toolkit.</p>
      
      <p><strong>Key Findings:</strong></p>
      <ul>
        <li><strong>Noise Characteristics:</strong> Updated accelerometer and gyroscope noise densities and random-walk coefficients from Allan plots and parameter estimates</li>
        <li><strong>Sampling Rate:</strong> Initial IMU data rate was approximately 30 Hz - significantly lower than the 50 Hz minimum typically expected for onboard state estimation, and well below the more common 200 Hz+ range</li>
        <li><strong>Improvements:</strong> After adjustments, the current maximum rate reached 90 Hz, and additional investigation is ongoing to further increase and stabilize the sampling frequency</li>
      </ul>
      
      <div style="margin: 24px 0; padding: 20px; background: #F8F9FA; border-radius: 12px; border-left: 4px solid #0668E1;">
        <img src="https://berkeleydrone.github.io/images/docs/accel_noise.png" alt="Accelerometer Calibration" style="width: 100%; border-radius: 8px; margin-bottom: 12px;">
        <p style="margin: 0 0 20px 0; font-size: 0.875rem; color: #65676B;"><strong>Figure 2:</strong> Calibration results - accelerometer</p>
        
        <img src="https://berkeleydrone.github.io/images/docs/gyro_noise.png" alt="Gyroscope Calibration" style="width: 100%; border-radius: 8px; margin-bottom: 12px;">
        <p style="margin: 0 0 20px 0; font-size: 0.875rem; color: #65676B;"><strong>Figure 3:</strong> Calibration results - gyroscope</p>
        
        <img src="https://berkeleydrone.github.io/images/docs/summary_noise.png" alt="Calibration Summary" style="width: 100%; border-radius: 8px; margin-bottom: 12px;">
        <p style="margin: 0; font-size: 0.875rem; color: #65676B;"><strong>Figure 4:</strong> Calibration results - summary</p>
      </div>
      
      <h3>Camera Calibration</h3>
      <p>Camera intrinsic calibration and IMU–camera extrinsic calibration were performed using the <strong>Kalibr toolbox</strong> (ROS1). Multiple calibration bags were collected while moving the drone and checkerboard, producing accurate estimates of:</p>
      <ul>
        <li>Focal lengths</li>
        <li>Principal points</li>
        <li>Distortion parameters</li>
        <li>Rigid transformation T<sub>IC</sub> between the IMU and camera frames</li>
      </ul>
      
      <div style="margin: 24px 0; padding: 20px; background: #F8F9FA; border-radius: 12px; border-left: 4px solid #0668E1;">
        <img src="https://berkeleydrone.github.io/images/docs/calibrated_images.png" alt="Calibrated Images" style="width: 100%; border-radius: 8px; margin-bottom: 12px;">
        <p style="margin: 0; font-size: 0.875rem; color: #65676B;"><strong>Figure 1:</strong> Demonstration of calibrated images</p>
      </div>
      
      <p>The reprojection errors, pose visualizations, and outlier-rejection plots are included in the calibration output.</p>
      
      <h3>Impact</h3>
      <p>These calibrations ensure that:</p>
      <ul>
        <li>The visual pipeline can perform <strong>consistent geometric reconstruction</strong></li>
        <li>Visual–inertial fusion is <strong>properly synchronized</strong></li>
        <li>State estimation achieves the accuracy required for high-speed autonomous flight</li>
      </ul>
    `
  },

  // ========== 2: Perception ==========
  {
    tag: 'Perception',
    title: 'Perception Module',
    subtitle: 'ArUco Marker Detection & Tracking',
    cardBg: '',
    cardGradient: 'green',
    bg: '',
    content: `
      <h3>Overview</h3>
      <p>The <strong>Perception Module</strong> is responsible for processing raw camera images to detect and localize visual targets (ArUco markers and gates) in the drone's environment. It acts as the <strong>sensory processing layer</strong> that bridges <strong>Hardware</strong> (camera sensors) and <strong>Planning</strong> (high-level decision making), providing real-time visual feedback for autonomous navigation and tracking.</p>
      
      <h3>High-Level Architecture</h3>
      <p>The perception module provides multiple detection strategies:</p>
      <ul>
        <li><strong>ArUco Detector Node</strong> — Detects and tracks ArUco fiducial markers, providing pixel coordinates and 3D pose estimates</li>
        <li><strong>Gate Detector Node</strong> — Detects racing gates using color-based segmentation and contour detection</li>
        <li><strong>Dummy Image Publisher</strong> — Testing utility that publishes images from disk for offline development</li>
      </ul>
      <p>Each detector runs as a standalone ROS2 node and publishes detection results to planning nodes, which use this information to generate desired trajectories.</p>
      
      <h3>1. ArUco Detector Node</h3>
      <p><strong>Purpose:</strong> Detects ArUco markers in camera images and publishes their pixel coordinates and 3D pose in the camera frame. ArUco markers are binary square fiducial markers designed for fast, robust detection and pose estimation, making them ideal for drone tracking applications.</p>
      
      <p><strong>Configuration Parameters:</strong></p>
      <ul>
        <li><code>camera_topic</code> — Input image topic (default: <code>/arducam/image_raw</code>)</li>
        <li><code>camera_info_topic</code> — Camera calibration topic (default: <code>/arducam/camera_info</code>)</li>
        <li><code>aruco_dict_id</code> — ArUco dictionary ID (default: 4 = DICT_4X4_50)</li>
        <li><code>marker_size</code> — Physical marker size in meters (default: 0.15 m)</li>
        <li><code>target_marker_id</code> — Specific marker ID to track (default: -1 = track any marker)</li>
      </ul>
      
      <p><strong>Detection Algorithm:</strong></p>
      <ul>
        <li><strong>Marker Detection:</strong> Uses <code>cv2.aruco.detectMarkers()</code> to detect markers using dictionary matching</li>
        <li><strong>Center Computation:</strong> Computes marker center as mean of corner coordinates</li>
        <li><strong>3D Pose Estimation:</strong> Uses <code>cv2.solvePnP()</code> with IPPE_SQUARE method for accurate pose from single marker</li>
        <li><strong>Quaternion Conversion:</strong> Converts rotation vector to quaternion using Shepperd's method</li>
      </ul>
      
      <p><strong>Inputs:</strong></p>
      <ul>
        <li><code>/arducam/image_raw</code> (sensor_msgs/Image) — Raw camera images (BGR8 encoding)</li>
        <li><code>/arducam/camera_info</code> (sensor_msgs/CameraInfo) — Camera intrinsic parameters</li>
      </ul>
      
      <p><strong>Outputs:</strong></p>
      <ul>
        <li><code>/aruco_detector/marker_pixel</code> (geometry_msgs/PointStamped) — Marker center in pixel coordinates</li>
        <li><code>/aruco_detector/marker_pose</code> (geometry_msgs/PoseStamped) — Marker 3D pose in camera frame</li>
        <li><code>/aruco_detector/debug_image</code> (sensor_msgs/Image) — Debug visualization with marker overlay</li>
        <li><code>/aruco_detector/detected</code> (std_msgs/Bool) — Detection status</li>
      </ul>
      
      <h3>2. Gate Detector Node</h3>
      <p><strong>Purpose:</strong> Detects racing gates in camera images using color-based segmentation and contour detection. Gates are detected by their distinctive color (LAB color space filtering) and geometric shape (quadrilateral approximation). Includes visual tracking (CSRT tracker) to maintain detection during temporary occlusions.</p>
      
      <p><strong>Configuration Parameters:</strong></p>
      <ul>
        <li><code>lab_lower</code> — LAB color lower bound (default: [19, 139, 100])</li>
        <li><code>lab_upper</code> — LAB color upper bound (default: [126, 163, 119])</li>
        <li><code>gate_width</code> — Physical gate width in meters (default: 1.4 m)</li>
        <li><code>gate_height</code> — Physical gate height in meters (default: 1.4 m)</li>
        <li><code>dilation_factor</code> — Morphological dilation kernel size (default: 5)</li>
        <li><code>downsample_factor</code> — Image downsampling factor (default: 10)</li>
      </ul>
      
      <p><strong>Detection Algorithm:</strong></p>
      <ul>
        <li><strong>Color Segmentation:</strong> Convert to LAB color space and apply color filter to detect gate color</li>
        <li><strong>Morphological Processing:</strong> Downsample mask (1920×1080 → 192×108), dilate to merge nearby regions</li>
        <li><strong>Gate Detection:</strong> Find largest connected component (gate frame), compute convex hull, find largest "hole" inside gate (gate opening), approximate hole as quadrilateral</li>
        <li><strong>Visual Tracking:</strong> If detection fails, use CSRT tracker to follow previously detected corners</li>
        <li><strong>Pose Estimation:</strong> If 4 corners detected, use <code>cv2.solvePnP()</code> to estimate gate pose</li>
      </ul>
      
      <p><strong>Why LAB Color Space:</strong> LAB color space is perceptually uniform, making it better for color-based segmentation than RGB. L channel represents lightness, A channel green-red axis, B channel blue-yellow axis.</p>
      
      <p><strong>Inputs:</strong></p>
      <ul>
        <li><code>/arducam/image_raw</code> (sensor_msgs/Image) — Raw camera images</li>
        <li><code>/arducam/camera_info</code> (sensor_msgs/CameraInfo) — Camera calibration parameters</li>
      </ul>
      
      <p><strong>Outputs:</strong></p>
      <ul>
        <li><code>/gate_detector/gate_mask</code> (sensor_msgs/Image) — Binary mask showing detected gate region</li>
        <li><code>/gate_detector/gate_overlay</code> (sensor_msgs/Image) — Original image with gate corners overlaid</li>
        <li><code>/gate_detector/gate_pose</code> (geometry_msgs/PoseStamped) — Gate 3D pose in camera frame</li>
      </ul>
      
      <h3>Connection to Planning and Control</h3>
      <p><strong>Perception → Planning:</strong></p>
      <ul>
        <li><strong>ArUco Detector</strong> publishes <code>/aruco_detector/marker_pixel</code> and <code>/aruco_detector/marker_pose</code></li>
        <li><strong>ArUco Tracker Planner</strong> subscribes and computes pixel error and distance error</li>
        <li>Planner generates velocity commands: <code>vel_y = -gain_x × error_x</code>, <code>vel_x = gain_z × distance_error</code></li>
        <li><strong>Gate Detector</strong> publishes <code>/gate_detector/gate_pose</code> for gate navigation tasks</li>
      </ul>
      
      <p><strong>Feedback Loop:</strong></p>
      <ul>
        <li><strong>Perception</strong> detects marker/gate position in image</li>
        <li><strong>Planning</strong> computes desired velocity to center marker</li>
        <li><strong>Control</strong> executes velocity commands via motors</li>
        <li><strong>Hardware</strong> moves drone; camera captures new image</li>
        <li>Loop closes: perception → planning → control → motion → perception</li>
      </ul>
      
      <h3>Real-World Challenges and Robustness</h3>
      <p><strong>Motion Blur:</strong></p>
      <ul>
        <li><strong>Problem:</strong> Fast drone motion causes image blur, reducing detection accuracy</li>
        <li><strong>Mitigation:</strong> Shorter exposure time, higher frame rate, visual tracking (CSRT tracker in gate detector), motion prediction</li>
        <li><strong>Improvement:</strong> Add Kalman filter to ArUco detector for motion prediction</li>
      </ul>
      
      <p><strong>Lighting Changes:</strong></p>
      <ul>
        <li><strong>Problem:</strong> Outdoor lighting varies with time of day, weather, shadows. Color-based detection particularly sensitive</li>
        <li><strong>Mitigation:</strong> Fixed LAB thresholds (calibrated for specific lighting)</li>
        <li><strong>Improvement:</strong> Implement adaptive color threshold adjustment based on image statistics</li>
      </ul>
      
      <p><strong>Distance and Scale Variations:</strong></p>
      <ul>
        <li><strong>Problem:</strong> Detection accuracy degrades with distance (targets become small). Pose estimation error increases quadratically with distance</li>
        <li><strong>Mitigation:</strong> Fixed marker size (0.15 m) and gate size (1.4 m × 1.4 m)</li>
        <li><strong>Improvement:</strong> Implement multi-scale detection or adaptive size estimation</li>
      </ul>
      
      <p><strong>Computational Performance:</strong></p>
      <ul>
        <li><strong>Problem:</strong> High-resolution images (1920×1080) require significant processing time</li>
        <li><strong>Mitigation:</strong> Gate detector downsamples by factor of 10, ArUco detector processes full resolution</li>
        <li><strong>Improvement:</strong> Add ROI processing and GPU acceleration for ArUco detector</li>
      </ul>
      
      <h3>Key Packages and Dependencies</h3>
      <table>
        <tr><th>Package</th><th>Usage</th></tr>
        <tr><td><code>rclpy</code></td><td>ROS2 Python client library; Node class, subscriptions, publishers</td></tr>
        <tr><td><code>cv_bridge</code></td><td>Converts ROS2 Image messages to/from OpenCV numpy arrays</td></tr>
        <tr><td><code>cv2.aruco</code></td><td>ArUco marker detection: detectMarkers(), drawDetectedMarkers()</td></tr>
        <tr><td><code>cv2.solvePnP</code></td><td>Perspective-n-Point pose estimation</td></tr>
        <tr><td><code>cv2.legacy.TrackerCSRT</code></td><td>Visual tracking for gate corners (Discriminative Correlation Filter)</td></tr>
        <tr><td><code>numpy</code></td><td>Numerical operations (mean, reshape, array manipulation)</td></tr>
      </table>
      
      <h3>Testing and Validation</h3>
      <p><strong>Recommended Procedures:</strong></p>
      <ul>
        <li><strong>Unit Tests:</strong> Test ArUco detection on synthetic images with known marker positions</li>
        <li><strong>ROS2 Integration Tests:</strong> Publish test images via dummy image publisher, verify detection topics at expected rate</li>
        <li><strong>Camera Calibration Validation:</strong> Verify calibration received and cached, validate reprojection error</li>
        <li><strong>Offline Testing:</strong> Record ROS2 bag with camera images, replay and compare detections to ground truth</li>
        <li><strong>Real-Time Testing:</strong> Monitor processing latency (should match camera frame rate), test under various lighting conditions</li>
        <li><strong>Failure Mode Testing:</strong> Test with marker/gate outside field of view, partial occlusion, extreme viewing angles</li>
      </ul>
    `
  },

  // ========== 3: Planning ==========
  {
    tag: 'Planning',
    title: 'Planning Module',
    subtitle: 'Error Computation & Trajectory Planning',
    cardBg: '',
    cardGradient: 'blue',
    bg: '',
    content: `
      <h3>Overview</h3>
      <p>The <strong>Planning Module</strong> is responsible for generating desired drone poses (position, orientation, and velocity setpoints) based on pre-planned trajectories, sensor feedback, or real-time perception data. It acts as the <strong>high-level decision layer</strong> that bridges <strong>Perception</strong> (what the drone sees) and <strong>Control</strong> (how the drone executes commands).</p>
      
      <h3>High-Level Architecture</h3>
      <p>The planning module provides multiple planning strategies:</p>
      <ul>
        <li><strong>Autonomous Planner</strong> — Follows pre-recorded waypoint trajectories from CSV files using KD-tree nearest-neighbor search</li>
        <li><strong>ArUco Tracker Planner</strong> — Dynamically tracks ArUco markers, computing desired poses to keep markers centered in the camera view</li>
        <li><strong>Sequential Planner</strong> — Simple waypoint-by-waypoint traversal without localization feedback</li>
        <li><strong>Manual Planner</strong> — Records pilot-controlled flight paths for later replay</li>
      </ul>
      <p>Each planner runs as a standalone ROS2 node and publishes desired poses to the <strong>Control Module</strong>, which then executes attitude and thrust commands via the flight controller.</p>
      
      <h3>1. Autonomous Planning Node</h3>
      <p><strong>Purpose:</strong> Loads a trajectory from a CSV file (x, y, z, yaw, t) and autonomously follows it by continuously publishing the next waypoint based on the drone's current position.</p>
      
      <p><strong>Configuration Parameters:</strong></p>
      <ul>
        <li><strong>Timer Rate:</strong> 10 Hz (0.1 s)</li>
        <li><strong>Path File:</strong> <code>src/planning/paths/course_1.csv</code></li>
        <li><strong>Output Frame:</strong> <code>map</code> frame</li>
        <li><strong>QoS:</strong> SENSOR_QOS for sensor data, COMMAND_QOS for critical commands</li>
      </ul>
      
      <p><strong>High-Level Logic:</strong></p>
      <ul>
        <li>Loads CSV path and constructs a KD-tree spatial index for fast nearest-neighbor queries (O(log n))</li>
        <li>Receives current drone pose from localization module</li>
        <li>On 10 Hz timer, queries planner to find closest path point and next point</li>
        <li>Publishes next point as desired pose</li>
        <li>Waits for <code>start_autonomous</code> signal before beginning trajectory execution</li>
      </ul>
      
      <p><strong>CSV Path Format:</strong></p>
      <ul>
        <li>Headers: <code>x, y, z, yaw, t</code></li>
        <li>Units: meters for position, radians for yaw, seconds for time</li>
        <li>Example: <code>0.0,0.0,1.0,0.0,0.0</code> (hover at 1m altitude)</li>
      </ul>
      
      <p><strong>Inputs:</strong></p>
      <ul>
        <li><code>/localization_msg</code> (DroneState) — Current drone position (x, y, z) from localization module</li>
        <li><code>start_autonomous</code> (Bool) — Signal to enable/disable autonomous flight</li>
      </ul>
      
      <p><strong>Outputs:</strong></p>
      <ul>
        <li><code>/desired_pose</code> (DroneState) — Desired position, orientation (quaternion), frame metadata</li>
      </ul>
      
      <h3>2. ArUco Tracker Planning Node</h3>
      <p><strong>Purpose:</strong> Dynamically tracks an ArUco marker in the camera view by computing velocity commands that center the marker in the image and maintain a fixed distance.</p>
      
      <p><strong>Configuration Parameters:</strong></p>
      <ul>
        <li><code>image_width</code>, <code>image_height</code> — Camera resolution (default: 1920×1080)</li>
        <li><code>gain_x</code>, <code>gain_y</code> — Pixel error to lateral/vertical velocity conversion (default: 0.002)</li>
        <li><code>gain_z</code> — Distance error to forward velocity conversion (default: 0.5)</li>
        <li><code>target_distance</code> — Desired distance to marker (default: 1.5 m)</li>
        <li><code>hover_altitude</code> — Altitude when marker not detected (default: 1.0 m)</li>
        <li><code>max_vel_xy</code>, <code>max_vel_z</code> — Velocity saturation limits (default: 0.5 m/s, 0.3 m/s)</li>
        <li><code>detection_timeout</code> — Max time without detection before switching to hover (default: 1.0 s)</li>
      </ul>
      
      <p><strong>Control Law:</strong></p>
      <ul>
        <li><strong>Pixel Error:</strong> <code>error_x = marker_pixel.x - (image_width / 2)</code></li>
        <li><strong>Velocity Computation:</strong> <code>vel_y = -gain_x × error_x</code> (lateral), <code>vel_z = -gain_y × error_y</code> (vertical), <code>vel_x = gain_z × distance_error</code> (forward)</li>
        <li><strong>Position Integration:</strong> <code>position_desired = position_current + velocity × dt</code></li>
        <li><strong>If marker detected:</strong> Compute lateral/vertical velocity to center marker, compute forward velocity to maintain distance</li>
        <li><strong>If marker lost:</strong> Switch to hover mode (zero velocity, maintain current position)</li>
      </ul>
      
      <p><strong>Inputs:</strong></p>
      <ul>
        <li><code>/aruco_detector/marker_pixel</code> (PointStamped) — Marker center in image pixel coordinates</li>
        <li><code>/aruco_detector/marker_pose</code> (PoseStamped) — Marker 3D pose in camera frame</li>
        <li><code>/aruco_detector/detected</code> (Bool) — Marker detection status</li>
        <li><code>/localization_msg</code> (DroneState) — Current drone state (position, orientation, velocity)</li>
      </ul>
      
      <p><strong>Outputs:</strong></p>
      <ul>
        <li><code>/desired_pose</code> (DroneState) — Desired position, orientation, and velocity setpoints</li>
      </ul>
      
      <h3>3. Sequential Planner Node</h3>
      <p><strong>Purpose:</strong> Simple waypoint publisher that sequentially outputs CSV waypoints without localization feedback. Useful for testing or GPS-denied scenarios with pre-programmed timing.</p>
      
      <p><strong>High-Level Logic:</strong></p>
      <ul>
        <li>Loads waypoints from CSV</li>
        <li>On 0.25 s timer, publishes next waypoint regardless of drone position</li>
        <li>Stops when all waypoints are published</li>
      </ul>
      
      <h3>4. Manual Planning Node</h3>
      <p><strong>Purpose:</strong> Records the drone's actual flight path during manual pilot control, producing a trajectory that can be replayed autonomously.</p>
      
      <p><strong>High-Level Logic:</strong></p>
      <ul>
        <li>Subscribes to <code>/local_position/pose</code> and appends each pose to recorded trajectory</li>
        <li>Can be extended to save trajectory to file for later autonomous playback</li>
      </ul>
      
      <h3>Connection to Perception and Control</h3>
      <p><strong>Planning ← Perception:</strong></p>
      <ul>
        <li>ArUco Tracker subscribes directly to <code>/aruco_detector/marker_pixel</code>, <code>/aruco_detector/marker_pose</code>, <code>/aruco_detector/detected</code></li>
        <li>Enables real-time reactive tracking</li>
      </ul>
      
      <p><strong>Planning ← Localization:</strong></p>
      <ul>
        <li>All planners subscribe to <code>/localization_msg</code> for drone state (position, orientation, velocity)</li>
        <li>Provides ground-truth feedback for trajectory correction and marker tracking baseline</li>
      </ul>
      
      <p><strong>Planning → Control:</strong></p>
      <ul>
        <li>All planners publish <code>/desired_pose</code> (DroneState with position and velocity setpoints)</li>
        <li>Control module reads this topic and generates attitude/thrust commands via cascade or PID control</li>
      </ul>
      
      <p><strong>Feedback Loop:</strong></p>
      <ul>
        <li>Control module's actual execution is monitored by localization</li>
        <li>If drone drifts off path, autonomous planner's next-point query corrects course</li>
        <li>If marker tracking overshoots, velocity commands are saturated and next control step corrects</li>
      </ul>
      
      <h3>ROS2 Topics and Messages</h3>
      <table>
        <tr><th>Node</th><th>Subscribes To</th><th>Publishes To</th></tr>
        <tr>
          <td><strong>Autonomous Planner</strong></td>
          <td><code>/localization_msg</code> (DroneState)<br><code>start_autonomous</code> (Bool)</td>
          <td><code>/desired_pose</code> (DroneState)</td>
        </tr>
        <tr>
          <td><strong>ArUco Tracker</strong></td>
          <td><code>/aruco_detector/marker_pixel</code><br><code>/aruco_detector/marker_pose</code><br><code>/aruco_detector/detected</code><br><code>/localization_msg</code></td>
          <td><code>/desired_pose</code> (DroneState)</td>
        </tr>
        <tr>
          <td><strong>Sequential Planner</strong></td>
          <td>(CSV file, no ROS2 subscriptions)</td>
          <td><code>/desired_pose</code> (DroneState)</td>
        </tr>
        <tr>
          <td><strong>Manual Planner</strong></td>
          <td><code>/local_position/pose</code> (PoseStamped)</td>
          <td>(Records internally, no publication)</td>
        </tr>
      </table>
      
      <h3>Key Packages and Dependencies</h3>
      <table>
        <tr><th>Package</th><th>Usage</th></tr>
        <tr><td><code>rclpy</code></td><td>ROS2 Python client library; Node class, subscriptions, publishers, timers</td></tr>
        <tr><td><code>autonomy_msgs</code></td><td>Custom DroneState message definition</td></tr>
        <tr><td><code>scipy.spatial.cKDTree</code></td><td>3D spatial indexing for fast nearest-neighbor search (O(log n) queries)</td></tr>
        <tr><td><code>tf_transformations</code></td><td>Converts Euler angles (roll, pitch, yaw) to quaternion orientation</td></tr>
        <tr><td><code>csv</code></td><td>CSV file parsing for loading waypoint paths</td></tr>
        <tr><td><code>numpy</code></td><td>Coordinate arrays and math operations</td></tr>
      </table>
      
      <h3>Implementation Details</h3>
      <p><strong>KD-Tree Query Performance:</strong></p>
      <ul>
        <li>Building the tree: O(n log n) once at startup</li>
        <li>Querying nearest neighbor: O(log n) per query</li>
        <li>For 100-waypoint trajectory, each query is ~7 operations</li>
        <li>At 10 Hz, overhead is negligible</li>
      </ul>
      
      <p><strong>Velocity Integration:</strong></p>
      <ul>
        <li>ArUco Tracker computes velocity commands but must convert to position setpoints</li>
        <li>Formula: <code>x_des = x_current + vel_x × dt</code></li>
        <li>With dt = 0.05 s, creates small incremental position changes</li>
      </ul>
      
      <p><strong>Quaternion Representation:</strong></p>
      <ul>
        <li>All orientation represented as quaternions (x, y, z, w) for ROS2 compatibility</li>
        <li>Conversion from yaw: <code>q = quaternion_from_euler(roll=0.0, pitch=0.0, yaw=yaw_des)</code></li>
      </ul>
      
      <h3>Potential Failure Modes</h3>
      <ul>
        <li><strong>Localization Dropout:</strong> If localization node crashes, planner freezes. <em>Mitigation: Add watchdog timer, transition to manual control after timeout.</em></li>
        <li><strong>Trajectory Complete:</strong> After reaching final waypoint, <code>find_next_point()</code> returns None. <em>Mitigation: Add null-check, publish hold position or land command.</em></li>
        <li><strong>ArUco Marker Lost:</strong> Marker disappears from view, planner switches to hover. <em>Mitigation: Already implemented with 1s timeout. Improvement: Add hysteresis to avoid rapid mode switching.</em></li>
        <li><strong>Pixel Error Exceeds Bounds:</strong> Marker detected but pixel coordinates outside image. <em>Mitigation: Add input validation, clamp pixel coordinates to valid range.</em></li>
        <li><strong>Frame ID Mismatch:</strong> Planning publishes in <code>map</code> frame, control expects <code>base_link</code>. <em>Mitigation: Add frame transformation layer, make frame_id configurable.</em></li>
      </ul>
      
      <h3>Future Improvements</h3>
      <ul>
        <li><strong>Dynamic Path Loading Service:</strong> Add ROS2 service to load different CSV files at runtime for mission replanning</li>
        <li><strong>Collision Avoidance:</strong> Subscribe to obstacle detection topics, implement local path replanning (velocity obstacles, potential field)</li>
        <li><strong>Geofencing:</strong> Define flight boundaries (min/max x, y, z) and validate all desired poses before publishing</li>
        <li><strong>Mission State Machine:</strong> Implement states (IDLE → ARM → TAKEOFF → WAYPOINT_FOLLOW → LAND → DISARM)</li>
        <li><strong>Adaptive Lookahead:</strong> Use velocity-dependent lookahead distance for smoother tracking at high speeds</li>
        <li><strong>Path Smoothing:</strong> Add spline interpolation (Bézier, cubic) for smooth curved paths, reduce jerk</li>
        <li><strong>Covariance-Aware Planning:</strong> Increase conservatism (wider margins, slower speeds) when localization uncertainty is high</li>
      </ul>
      
      <h3>Testing and Validation</h3>
      <p><strong>Recommended Procedures:</strong></p>
      <ul>
        <li><strong>Unit Tests:</strong> Test KD-tree queries with synthetic waypoints, test CSV loading with malformed files</li>
        <li><strong>Simulation Tests:</strong> Run planner nodes in SITL with Gazebo physics, inject sensor noise and dropouts</li>
        <li><strong>Hardware Bench Tests:</strong> Verify localization callback consistency, monitor CPU/memory usage</li>
        <li><strong>Flight Tests:</strong> Tethered → Simple rectangular path → ArUco tracking → Complex scenarios (GPS dropout, marker occlusion)</li>
      </ul>
    `
  },

  // ========== 4: Control ==========
  {
    tag: 'Control',
    title: 'Control Module',
    subtitle: 'PID Controller & Velocity Commands',
    cardBg: '',
    cardGradient: 'purple',
    bg: '',
    content: `
      <h3>Overview</h3>
      <p>The <strong>Control Module</strong> is responsible for converting high-level desired poses and velocities (from the Planning Module) into low-level RC (Radio Control) commands that drive the drone's motors via the flight controller. It acts as the <strong>closed-loop execution layer</strong> that bridges <strong>Planning</strong> (what the drone should do) and <strong>Hardware</strong> (how the motors respond).</p>
      
      <h3>High-Level Architecture</h3>
      <p>The control module provides multiple control strategies:</p>
      <ul>
        <li><strong>Autonomous Control Node</strong> — Full 6-DOF position and attitude control using cascade PID with localization feedback</li>
        <li><strong>Velocity Control Node</strong> — Simplified velocity-to-RC mapping for visual servoing (no localization required)</li>
        <li><strong>Manual Control Node</strong> — Pass-through relay for human pilot RC commands</li>
        <li><strong>Test Control Monitor</strong> — Real-time monitoring and debugging tool for safe testing without hardware</li>
      </ul>
      <p>Each controller runs as a standalone ROS2 node and publishes RC override commands to the <strong>Flight Controller</strong> (Betaflight via MSP), which then executes motor commands.</p>
      
      <h3>1. Autonomous Control Node</h3>
      <p><strong>Purpose:</strong> Implements a <strong>cascade PID controller</strong> that converts desired 3D positions, orientations, and velocities into attitude (roll, pitch, yaw rate) and thrust commands. Requires continuous localization feedback for closed-loop tracking.</p>
      
      <p><strong>Configuration Parameters:</strong></p>
      <ul>
        <li><strong>Control Frequency:</strong> 50 Hz (0.02 s period)</li>
        <li><strong>PID Gains (X-axis):</strong> KP=8.0, KI=0.0, KD=5.0</li>
        <li><strong>PID Gains (Y-axis):</strong> KP=8.0, KI=0.0, KD=5.0</li>
        <li><strong>PID Gains (Z-axis):</strong> KP=20.0, KI=0.0, KD=5.0 (altitude)</li>
        <li><strong>PID Gains (Yaw):</strong> KP=1.0, KI=0.0, KD=0.0</li>
        <li><strong>Drone Mass:</strong> 1.5 kg</li>
        <li><strong>Thrust Limits:</strong> [0.0, 1000.0] N</li>
      </ul>
      
      <p><strong>Control Law:</strong></p>
      
      <p><strong>Outer Loop (Position Control):</strong></p>
      <ul>
        <li>Compute position error: <code>pos_error = desired_pos - current_pos</code></li>
        <li>Compute velocity error: <code>vel_error = desired_vel - current_vel</code></li>
        <li>Desired acceleration: <code>r_ddot_des = KP × pos_error + KD × vel_error</code></li>
      </ul>
      
      <p><strong>Inner Loop (Attitude Computation):</strong></p>
      <ul>
        <li>Roll: <code>phi_des = (r_ddot_des_x × sin(yaw) - r_ddot_des_y × cos(yaw)) / g</code></li>
        <li>Pitch: <code>theta_des = (r_ddot_des_x × cos(yaw) + r_ddot_des_y × sin(yaw)) / g</code></li>
      </ul>
      
      <p><strong>Z-Axis Control (Thrust with Gravity Compensation):</strong></p>
      <ul>
        <li>Compute thrust with gravity: <code>U1 = (KP_z × pos_error[2] - KD_z × vz) / (cos(roll) × cos(pitch)) + (m × g) / (cos(roll) × cos(pitch))</code></li>
        <li>Clamp to limits: <code>U1 = clamp(U1, U1_min, U1_max)</code></li>
      </ul>
      
      <p><strong>Angle to PWM Mapping:</strong></p>
      <ul>
        <li><strong>Roll:</strong> 1500 + map(roll_deg, -30° to +30°, -500 to +500)</li>
        <li><strong>Pitch:</strong> 1500 + map(pitch_deg, -30° to +30°, -500 to +500)</li>
        <li><strong>Yaw:</strong> 1500 + map(yaw_rate, -45°/s to +45°/s, -500 to +500)</li>
        <li><strong>Throttle:</strong> map(thrust, 0 to 50 N, 1000 to 2000)</li>
      </ul>
      
      <h3>2. Velocity Control Node</h3>
      <p><strong>Purpose:</strong> Simplified control for <strong>visual servoing</strong> tasks (e.g., ArUco tracking) where only velocity commands are needed. Does NOT require localization—works purely on velocity feedback, making it suitable for GPS-denied or marker-tracking scenarios.</p>
      
      <p><strong>Configuration Parameters:</strong></p>
      <ul>
        <li><code>vel_to_rc_x</code> — Forward velocity gain (default: 100.0 PWM/(m/s), currently unused)</li>
        <li><code>vel_to_rc_y</code> — Lateral velocity gain (default: 1300.0 PWM/(m/s))</li>
        <li><code>vel_to_rc_z</code> — Vertical velocity gain (default: 150.0 PWM/(m/s), currently unused)</li>
        <li><code>rc_center_*</code> — Neutral PWM values (1500)</li>
        <li><code>rc_min</code>, <code>rc_max</code> — Safe PWM range [1320, 1680]</li>
        <li><code>enable_output</code> — Safety flag (default: false; must be set to true to publish commands)</li>
        <li><code>max_vel_command</code> — Maximum allowed velocity magnitude (1.0 m/s)</li>
        <li><code>timeout</code> — Command timeout duration (1.0 s)</li>
      </ul>
      
      <p><strong>Control Law:</strong></p>
      <ul>
        <li><strong>Lateral (Y):</strong> <code>rc_yaw = 1500 - (1300.0 × vel_y)</code>, clamped to [1320, 1680]</li>
        <li><strong>Forward (X):</strong> Currently disabled for safety (rc_pitch = 1500)</li>
        <li><strong>Vertical (Z):</strong> Currently disabled for safety (rc_throttle = 1000)</li>
        <li><strong>Roll:</strong> Currently disabled for safety (rc_roll = 1500)</li>
      </ul>
      
      <p><strong>Note:</strong> Currently configured for <strong>yaw-only control</strong> with lateral velocity mapping. Forward (X) and vertical (Z) channels are disabled for safety during testing.</p>
      
      <h3>Connection to Planning and Localization</h3>
      <p><strong>Control ← Planning:</strong></p>
      <ul>
        <li>Control module subscribes to <code>/desired_pose</code> published by any planner node</li>
        <li>Receives desired position, orientation (quaternion), and velocity setpoints</li>
        <li><strong>Autonomous Control</strong> uses all fields; <strong>Velocity Control</strong> uses only twist (velocity)</li>
      </ul>
      
      <p><strong>Control ← Localization:</strong></p>
      <ul>
        <li><strong>Autonomous Control</strong> subscribes to <code>/localization_msg</code> for closed-loop feedback</li>
        <li>Receives current position, orientation, velocity, and angular rates</li>
        <li>Computes error signals: <code>error = desired - current</code></li>
        <li><strong>Velocity Control</strong> does NOT require localization (open-loop velocity commands)</li>
      </ul>
      
      <p><strong>Control → Hardware (Flight Controller):</strong></p>
      <ul>
        <li>Control module publishes <code>/rc/override</code> (OverrideRCIn message)</li>
        <li>Flight controller (Betaflight) receives commands via MSP (MultiWii Serial Protocol)</li>
        <li>MSP override mode must be enabled in Betaflight (AUX3 switch, <code>msp_override_channels_mask = 47</code>)</li>
        <li>Flight controller runs inner-loop stabilization (rate control, attitude hold)</li>
        <li>Motor mixing and ESC signals are handled by flight controller firmware</li>
      </ul>
      
      <h3>ROS2 Topics and Messages</h3>
      <table>
        <tr><th>Node</th><th>Subscribes To</th><th>Publishes To</th></tr>
        <tr>
          <td><strong>Autonomous Control</strong></td>
          <td><code>/desired_pose</code> (DroneState)<br><code>/localization_msg</code> (DroneState)</td>
          <td><code>/rc/override</code> (OverrideRCIn)</td>
        </tr>
        <tr>
          <td><strong>Velocity Control</strong></td>
          <td><code>/desired_pose</code> (DroneState)</td>
          <td><code>/rc/override</code> (OverrideRCIn)<br><code>/control/debug</code> (String)</td>
        </tr>
        <tr>
          <td><strong>Manual Control</strong></td>
          <td><code>/rc/manual_input</code> (OverrideRCIn)</td>
          <td><code>/rc/override</code> (OverrideRCIn)</td>
        </tr>
        <tr>
          <td><strong>Test Monitor</strong></td>
          <td><code>/desired_pose</code><br><code>/rc/override</code><br><code>/control/debug</code></td>
          <td>(None, terminal only)</td>
        </tr>
      </table>
      
      <h3>PWM Channel Mapping</h3>
      <p>Standard RC channel mapping (Betaflight configuration):</p>
      <ul>
        <li><strong>Channel 1 (Roll):</strong> 1500 = level, &lt;1500 = left, &gt;1500 = right</li>
        <li><strong>Channel 2 (Pitch):</strong> 1500 = level, &lt;1500 = backward, &gt;1500 = forward</li>
        <li><strong>Channel 3 (Throttle):</strong> 1000 = minimum, 2000 = maximum</li>
        <li><strong>Channel 4 (Yaw):</strong> 1500 = no rotation, &lt;1500 = CCW, &gt;1500 = CW</li>
        <li><strong>Channel 5 (AUX1):</strong> Arm switch (1000 = armed in this configuration)</li>
        <li><strong>Channel 6 (AUX2):</strong> Flight mode (1500 = angle mode, 990 = MSP override mode)</li>
      </ul>
      
      <h3>Implementation Details</h3>      
      <p><strong>Quaternion to Euler Conversion:</strong></p>
      <p>Control nodes receive orientation as quaternions from <code>DroneState.pose.pose.orientation</code>. The conversion to Euler angles (roll, pitch, yaw) follows the standard aerospace convention (ZYX Euler angles).</p>
      
      <p><strong>Gravity Compensation:</strong></p>
      <p>The thrust command must counteract gravity plus accelerate upward. The division by <code>cos(roll) * cos(pitch)</code> accounts for the fact that thrust is applied along the body Z-axis, not the world Z-axis. At large tilt angles, more thrust is needed to maintain altitude.</p>
      
      <p><strong>Control Loop Timing:</strong></p>
      <ul>
        <li><strong>Autonomous Control:</strong> 50 Hz (0.02 s period)</li>
        <li><strong>Velocity Control:</strong> 50 Hz (0.02 s period)</li>
        <li><strong>Flight Controller (Betaflight):</strong> Typically 500-1000 Hz internal rate control</li>
      </ul>
      <p>The control nodes publish at 50 Hz because localization updates at ~50 Hz (limited by sensor fusion rate), which is sufficient for outer-loop position control (bandwidth ~1-2 Hz). The flight controller handles fast inner-loop stabilization (rate control at 500+ Hz).</p>
      
      <h3>Key Packages and Dependencies</h3>
      <table>
        <tr><th>Package</th><th>Usage</th></tr>
        <tr><td><code>rclpy</code></td><td>ROS2 Python client library; Node class, subscriptions, publishers, timers</td></tr>
        <tr><td><code>autonomy_msgs</code></td><td>Custom DroneState message definition</td></tr>
        <tr><td><code>mavros_msgs</code></td><td>Standard OverrideRCIn message for RC channel override</td></tr>
        <tr><td><code>geometry_msgs</code></td><td>Standard ROS2 messages (PoseStamped, TwistStamped, Quaternion)</td></tr>
        <tr><td><code>numpy</code></td><td>Numerical operations (sin, cos, arctan2, arcsin, clamp, arrays)</td></tr>
      </table>
      
      <h3>Potential Failure Modes</h3>
      <ul>
        <li><strong>Localization Dropout:</strong> If EKF crashes or GPS signal lost, control loop freezes. <em>Mitigation: Add watchdog timer and transition to manual control or safe landing.</em></li>
        <li><strong>Gimbal Lock:</strong> At pitch = ±90°, Euler angle representation becomes singular. <em>Mitigation: Use quaternion-based attitude control.</em></li>
        <li><strong>Thrust Saturation:</strong> Computed thrust exceeds motor capacity. <em>Mitigation: Already clamped; log warning when saturation occurs.</em></li>
        <li><strong>Yaw Wraparound:</strong> Error computed incorrectly at ±180°. <em>Mitigation: Wrap yaw error to [-π, π].</em></li>
        <li><strong>MSP Override Not Enabled:</strong> Control publishes commands, but Betaflight ignores them. <em>Mitigation: Verify AUX3 switch enables MSP override mode.</em></li>
      </ul>
      
      <h3>Future Improvements</h3>
      <ul>
        <li><strong>Anti-Windup:</strong> Implement conditional integration or back-calculation for integral control</li>
        <li><strong>Quaternion-Based Control:</strong> Eliminate gimbal lock with quaternion error formulation</li>
        <li><strong>Feedforward Commands:</strong> Add desired acceleration feedforward for aggressive trajectories</li>
        <li><strong>Dynamic Reconfigure:</strong> Enable runtime PID gain tuning without node restart</li>
        <li><strong>State Machine:</strong> Implement flight modes (DISARMED → ARMED → TAKEOFF → AUTONOMOUS → LAND)</li>
        <li><strong>Command Timeout:</strong> Add watchdog timer for safety (switch to hover if no commands received)</li>
        <li><strong>Thrust Curve Calibration:</strong> Implement nonlinear thrust mapping from motor bench tests</li>
      </ul>
    `
  },

  // ========== 5: Workflow ==========
  {
    tag: 'Workflow',
    title: 'Development Workflow',
    subtitle: 'Software Development & Debugging Guide',
    cardBg: 'https://berkeleydrone.github.io/images/docs/pipeline_diagram.png',
    cardGradient: '',
    bg: 'https://berkeleydrone.github.io/images/docs/pipeline_diagram.png',
    content: `
      <h3>Overview</h3>
      <p>This document summarizes the work done so far in three development areas: hardware setup and testing, sensor calibration, and task implementation. For each area there is a short high-level summary followed by concrete examples and step-by-step procedures used while developing and validating the system.</p>
      
      <div style="margin: 24px 0; padding: 20px; background: #F8F9FA; border-radius: 12px; border-left: 4px solid #0668E1;">
        <img src="https://berkeleydrone.github.io/images/docs/pipeline_diagram.png" alt="Pipeline Diagram" style="width: 100%; border-radius: 8px; margin-bottom: 12px;">
        <p style="margin: 0; font-size: 0.875rem; color: #65676B;"><strong>Figure 1:</strong> ArUco Marker tracking pipeline diagram</p>
      </div>
      
      <h3>Step 1: Hardware Setup and Testing</h3>
      <p>We assembled and bench-tested the aircraft powertrain, flight controller, radios, and data links. Basic electrical and functional checks were performed before any flight attempts. The goal at this stage was to validate wiring, power distribution, and that actuators and sensors respond to commands predictably.</p>
      
      <p><strong>Detailed Steps:</strong></p>
      <ul>
        <li><strong>Visual inspection:</strong> Confirm frame integrity, motor/prop orientation, connector seating, and secure mounting of the flight controller and sensors</li>
        <li><strong>Power checks:</strong> Measure battery voltage and PDU rail voltages with a multimeter. Confirm no shorts and expected voltages under no-load and light-load conditions</li>
        <li><strong>ESC/motor bench test:</strong> Spin each motor on the bench using low-throttle commands from the flight controller (with props removed). Arm the controller in safe bench mode and verify smooth RPM increase</li>
        <li><strong>ESC calibration:</strong> Follow ESC vendor procedure — power on ESCs at full throttle then lower to minimum, or use flight controller's calibration utility</li>
        <li><strong>Telemetry and RC link test:</strong> Verify RC receiver channels, telemetry radio link, and ground-station connectivity. Use <code>lsusb</code> or <code>dmesg</code> to confirm serial devices appear</li>
        <li><strong>Smoke and thermal check:</strong> Run short powered tests and inspect for abnormal heating or smells</li>
        <li><strong>Logging + photographs:</strong> Record bench test results and configuration photos in the team log</li>
      </ul>
      
      <p><strong>Why This Matters:</strong> Hardware validation is the foundation of a reliable flight system. Skipping or rushing bench tests often leads to in-flight failures that put the vehicle at risk and waste development time.</p>
      
      <p><strong>Future Improvements:</strong></p>
      <ul>
        <li><strong>Automated health checks:</strong> Write scripts that cycle through all motors, checks sensor ranges, and validates radio links without manual intervention</li>
        <li><strong>Vibration analysis:</strong> Add high-speed video or accelerometer data capture during motor spin tests to detect imbalances early</li>
        <li><strong>Redundancy testing:</strong> Validate failover behavior (e.g., power loss to one ESC, automatic recovery)</li>
        <li><strong>Environmental testing:</strong> Test across temperature ranges, humidity, and outdoor EMI if the platform will operate in those conditions</li>
      </ul>
      
      <p><strong>Recommended Tools:</strong></p>
      <ul>
        <li><strong>Multimeter</strong> (Fluke 87 or equivalent): Precision voltage and resistance checks</li>
        <li><strong>Oscilloscope</strong> (entry-level, 100 MHz): Inspect PWM duty cycles, SPI/I2C signal integrity, and power rail ripple</li>
        <li><strong>Power supply</strong> (30V, 10+ A adjustable): Simulate battery voltage and test under various load conditions</li>
        <li><strong>Motor RPM tachometer:</strong> Non-contact RPM verification during bench spin tests</li>
        <li><strong>Thermal camera</strong> (e.g., FLIR): Spot overheating components (resistors, voltage regulators, ESCs)</li>
        <li><strong>MAVProxy or QGroundControl:</strong> Free ground station software to monitor telemetry and system health</li>
      </ul>
      
      <h3>Step 2: Sensor Calibration</h3>
      <p>We calibrated IMU, magnetometer, barometer, and camera to ensure reliable state estimation. Calibration included collecting sensor data in controlled maneuvers, computing biases and scale factors, and updating the flight controller and estimator configuration files with the resulting offsets.</p>
      
      <p><strong>Detailed Steps:</strong></p>
      <ul>
        <li><strong>IMU (accelerometer + gyro) calibration:</strong> Power the vehicle on a vibration-isolated bench (props off). Run the controller's built-in calibrator to compute bias and scale. Validate by checking that stationary orientation reports near-zero angular rates</li>
        <li><strong>Magnetometer (compass) calibration:</strong> Perform full rotation / figure-eight motions away from large ferrous structures. Fit hard/soft iron corrections and update offset matrix in flight controller configuration</li>
        <li><strong>Barometer / altimeter zeroing:</strong> Record barometer readings on bench and set local sea-level offset. Cross-check altitude changes against known height references</li>
        <li><strong>Camera / vision sensor calibration:</strong> Capture calibration target images and run standard camera calibration (e.g., OpenCV <code>calibrateCamera</code>) to produce intrinsics (focal length, principal point) and distortion coefficients</li>
        <li><strong>Calibration data management:</strong> Save calibration outputs (offsets, covariance updates, YAML/JSON files) and note the date and conditions</li>
      </ul>
      
      <p><strong>Why This Matters:</strong> Sensors have inherent biases, scale factor errors, and non-orthogonality. Without calibration, even small biases in the gyro accumulate into large orientation errors within seconds. Proper calibration ensures the state estimator receives high-quality measurements and converges reliably.</p>
      
      <p><strong>Future Improvements:</strong></p>
      <ul>
        <li><strong>In-flight re-calibration:</strong> Implement adaptive Kalman filter that refines IMU and mag biases during flight</li>
        <li><strong>Temperature compensation:</strong> Collect calibration data across wide temperature range and build polynomial bias correction tables</li>
        <li><strong>Magnetic field mapping:</strong> Pre-survey flight area with magnetometer to identify local magnetic anomalies</li>
        <li><strong>Camera auto-calibration:</strong> Use visual odometry or bundle adjustment to refine camera intrinsics on-the-fly</li>
        <li><strong>Cross-sensor validation:</strong> Compare accelerometer orientation estimates against visual horizon detection</li>
      </ul>
      
      <p><strong>Recommended Tools:</strong></p>
      <ul>
        <li><strong>Precision test stand:</strong> Vibration-isolated table, spirit level for repeatable calibration geometry</li>
        <li><strong>Calibration targets:</strong> Checkerboard patterns (for camera), Helmholtz coils or compass (for magnetometer)</li>
        <li><strong>MATLAB / Python + OpenCV:</strong> Implement least-squares fitting for IMU biases, mag correction, and camera distortion</li>
        <li><strong>ROS bag recorder:</strong> Log raw sensor data during calibration maneuvers for offline processing</li>
        <li><strong>Kalman filter framework:</strong> ArduPilot's EKF3, PX4's EKF2, or custom Python filterpy</li>
        <li><strong>Allan Variance tools:</strong> Measure sensor noise characteristics and bias instability</li>
      </ul>
      
      <h3>Step 3: Task Implementation</h3>
      <p>With hardware validated and sensors calibrated, we implemented the mission-level tasks: state estimation integration, low-level control verification, and higher-level mission scripts (waypoint navigation, automated maneuvers). Each task was developed incrementally: simulation and logged replay where possible, then bench tests, then limited tethered or short flights.</p>
      
      <p><strong>Detailed Steps:</strong></p>
      <ul>
        <li><strong>State estimation and controller integration:</strong> Integrate sensor streams into estimator (EKF/UKF), verify state publishers, and check covariance outputs. Run estimator in SITL or with logged bag files to confirm consistent position/attitude estimates</li>
        <li><strong>Control loop tuning:</strong> Tune PID/P gains on per-axis basis using step responses (small commanded attitude changes) and log analysis. Record before/after response plots</li>
        <li><strong>Mission scripting / autonomy:</strong> Implement mission definitions (YAML/JSON) describing waypoints, loiter times, and actions. Example: takeoff to 2 m, fly square waypoints at 1 m/s, return and land</li>
        <li><strong>Testing progression:</strong> Run missions first in simulation (SITL) or replay logs. Bench-test actuators with mission script in safe mode (props removed). Progress to short, restricted flights once bench and tether tests pass</li>
        <li><strong>Data logging and post-flight analysis:</strong> Enable full onboard logging. After each test/flight, download logs and analyze sensor fusion, control outputs, and trajectory tracking</li>
      </ul>
      
      <p><strong>Why This Matters:</strong> Task implementation is where the system must perform its intended mission reliably. Without well-integrated state estimator, tuned controllers, and validated mission logic, the vehicle will either diverge unstably or behave unpredictably.</p>
      
      <p><strong>Future Improvements:</strong></p>
      <ul>
        <li><strong>Adaptive control:</strong> Implement gain scheduling or MPC to adjust gains based on airspeed, wind, or battery voltage</li>
        <li><strong>Onboard machine learning:</strong> Train lightweight neural networks (TensorFlow Lite) to learn wind patterns or sensor biases in real time</li>
        <li><strong>Distributed estimation:</strong> Add secondary controllers that run parallel state estimators with voting/fusion logic for fault tolerance</li>
        <li><strong>Trajectory optimization:</strong> Plan optimal paths (fuel/time efficient) accounting for wind, no-fly zones, and dynamics constraints</li>
        <li><strong>Multi-vehicle coordination:</strong> Extend mission planning to swarms with inter-vehicle communication</li>
        <li><strong>Sim-to-real transfer:</strong> Use reinforcement learning to train control policies in simulation and deploy to real platform</li>
      </ul>
      
      <p><strong>Robustness Testing:</strong></p>
      <ul>
        <li><strong>Controller robustness:</strong> Command attitude steps and measure settling time. Inject wind gusts and verify disturbance rejection</li>
        <li><strong>Mission robustness:</strong> Simulate GPS drop-out (10-60 seconds) and verify vehicle maintains attitude and recovers position estimation</li>
        <li><strong>System integration:</strong> Test cold start vs warm start bias. Verify logging consistency and parameter reloading</li>
        <li><strong>Actuator saturation:</strong> Command aggressive rates that saturate motors and verify no integral windup</li>
      </ul>
      
      <p><strong>Recommended Tools:</strong></p>
      <ul>
        <li><strong>Simulation:</strong> Gazebo (ROS integration, physics), jMAVSim (lightweight HITL), Airsim (high-fidelity visuals), or custom Python simulator</li>
        <li><strong>Hardware-in-the-loop:</strong> Connect real flight controller to PC running Gazebo via serial/network</li>
        <li><strong>Instrumented test vehicle:</strong> GPS/RTK base station, onboard logging (microSD or telemetry), secondary safety receiver</li>
        <li><strong>Analysis:</strong> Plotly or Matplotlib (trajectory plotting), MATLAB (Bode plots), rosbag analysis (synchronous replay)</li>
        <li><strong>Performance metrics:</strong> SLAM benchmarking (evo, TUM evaluation scripts), custom Python scripts for waypoint tracking error</li>
        <li><strong>Safety:</strong> Real-time monitoring dashboard, kill-switch hardware, geofence validation</li>
      </ul>
      
      <h3>Summary</h3>
      <p>Each step builds on the previous one:</p>
      <ul>
        <li><strong>Hardware setup and testing</strong> validates the physical platform and communication links</li>
        <li><strong>Sensor calibration</strong> ensures the state estimator has high-quality measurements for reliable state fusion</li>
        <li><strong>Task implementation</strong> leverages calibrated sensors and validated hardware to execute autonomous missions</li>
      </ul>
      <p>Future work should focus on automating tests, adding redundancy, implementing adaptive algorithms, and validating robustness across a wider range of operating conditions.</p>
    `
  }
];
