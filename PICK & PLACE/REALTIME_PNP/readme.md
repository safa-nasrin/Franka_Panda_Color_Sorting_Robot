
# 🤖 NeoFlux: Vision-Based Pick and Place Robot (MyCobot 280 Pi)

This repository contains the core control software for the NeoFlux vision-guided robotic sorting system. It uses OpenCV for color and contour detection, an automated Perspective Transform matrix for precise real-world coordinate mapping, and ROS 2 for a live 3D Digital Twin visualization in RViz.

---

## 🧠 Core Implementation: Phase 2 - Real-Time AI Pick & Place
This phase integrates computer vision (OpenCV) with hardware kinematics to allow the physical MyCobot to autonomously detect, target, and manipulate physical objects in real-time, controlled via a local Flask web interface.

### 📐 The Vision-to-Hardware Calibration Workflow
To ensure millimeter-perfect accuracy, we implemented a precise 4-point perspective transformation and physical offset tuning. 

**The Setup & Calibration Process:**
1. **Camera Orientation Check:** We verified the live feed alignment. Because the physical camera is mounted sideways, a 90-degree counter-clockwise rotation (`cv2.ROTATE_90_COUNTERCLOCKWISE`) is applied directly in the code to perfectly align the software's perspective with the robot's physical workspace.
2. **Pixel Extraction:** We captured the exact 2D pixel coordinates `(x, y)` of 4 boundary points directly from the camera's live feed.
3. **Hardware Mapping:** We physically jogged the robot arm to those exact 4 boundary locations in the real world and recorded the physical mechanical coordinates `(mm)`.
4. **Integration Verification:** We linked these two datasets using `cv2.getPerspectiveTransform` and ran a standalone test script to command the robot to those calculated points, verifying if it moved to the exact physical locations.
5. **Offset Tuning:** If the robot slightly missed the true physical targets due to camera mounting angles or lens distortion, we calculated and applied manual X/Y offsets to guarantee absolute precision.
6. **Code Integration:** The finalized points and tuned offsets were permanently integrated into the main control script.

### ⚙️ Key Technical Features
* **Flask Web Interface:** Streams a live, annotated MJPEG video feed to a web browser, allowing users to click buttons to trigger "Pick Red", "Pick Blue", "Home", or "Unlock" commands via ROS 2 topics.
* **Live Digital Twin Sync:** As the physical robot moves to pick up the real objects, the script actively publishes `JointState` and `MarkerArray` data. This renders the cubes, containers, and arm movements inside RViz in real-time.
* **Smart Trajectory Routing:** Implements distinct kinematic pathways based on object location. If a cube is located in the "Bottom Zone" (close to the robot base), the arm folds into a designated "Safe Tucked Pose" before dropping down to prevent physical collisions with itself or the camera mount.
* **Shape & Color Processing:** Uses HSV masking and contour analysis to differentiate between the target cubes (solid objects) and the placement containers (hollow boundaries), calculating the correct rotation angles for the gripper to grasp the blocks cleanly.

---

## 📦 Setup Instructions for Evaluator

To safely run this project without interfering with the system's default Python packages, we use a shell script to automatically create an isolated virtual environment and install all necessary dependencies directly from the `requirements.txt` file.

**Run the Automated Setup:**
Open your terminal in this project directory and run the following commands:
```bash
chmod +x setup.sh
./setup.sh
```

## 🚀 How to Run the Project
To execute this phase smoothly, we use an automated launch script (`run_project.sh`) that safely sequences the ROS 2 bridge, builds the Digital Twin workspace, and boots the main Python AI script.

**Open your terminal and run:**
```bash
chmod +x run_project.sh
./run_project.sh
```

### 🔍 Under the Hood: The Launch Sequence
When you execute the command above, the script handles these three critical processes in the background to ensure a seamless experience:

**1. Communication Layer (Rosbridge Server)**
Starts the websocket server to bridge the gap between the Web Interface and ROS 2.
```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
```

**2. Hardware & Visualization Layer (Digital Twin)**
Builds the specific workspace packages and launches the RViz visualization.
```bash
(cd ~/colcon_ws && colcon build --packages-select mycobot_280pi mycobot_description && source install/setup.bash && ros2 launch ~/colcon_ws/digital_twin.launch.py) &
```

**3. Application Layer (Python AI Script)**
Runs the main intelligence in the foreground, managing the camera feed, Flask server, and pick-and-place logic.
```bash
python3 src/REALTIME_PNP/pick_place.py
```
---

## 🔮 Future Scope
1. **Voice-Controlled Manipulation:** Integrating NLP to trigger pick-and-place actions via voice commands.
2. **Cloud-Based Remote Digital Twin:** Enabling remote control and monitoring over a WAN using secure tunneling.
3. **Autonomous Defect Inspection:** Adding a quality control step where the AI inspects the object for damage before placement.
