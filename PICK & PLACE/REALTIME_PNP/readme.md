
# 🤖 NeoFlux: Vision-Based Pick and Place Robot (MyCobot 280 Pi)

This repository contains the core control software for the NeoFlux vision-guided robotic sorting system. It uses OpenCV for color and contour detection, an automated Perspective Transform matrix for precise real-world coordinate mapping, and ROS 2 for a live 3D Digital Twin visualization in RViz.

---

## 🧠 Core Implementation: Phase 2 - Real-Time AI Pick & Place
This phase integrates computer vision (OpenCV) with hardware kinematics to allow the physical MyCobot to autonomously detect, target, and manipulate physical objects in real-time, controlled via a local Flask web interface.

### 📐 The Vision-to-Hardware Calibration Workflow
To ensure millimeter-perfect accuracy, we implemented a precise 4-point perspective transformation and physical offset tuning. 

1. **Camera Alignment:** Applied a 90° CCW software rotation (`cv2.ROTATE_90_COUNTERCLOCKWISE`) to correct the sideways physical camera mount.
2. **Pixel Extraction:** Recorded the exact 2D `(x, y)` pixel coordinates of 4 boundary points from the live feed.
3. **Hardware Mapping:** Jogged the physical robot arm to those 4 boundary locations and recorded their real-world `(mm)` coordinates.
4. **Integration Verification:** Linked the pixel and hardware datasets using `cv2.getPerspectiveTransform` and tested movement accuracy.
5. **Offset Tuning:** Applied manual X/Y offsets to compensate for lens distortion and mounting angles.
6. **Code Integration:** Hardcoded the finalized matrices and tuned offsets directly into the main script.

### ⚙️ Key Technical Features
* **Flask Web Interface:** Streams a live, annotated MJPEG video feed with browser buttons that trigger ROS 2 commands (e.g., "Pick Red", "Home").
* **Live Digital Twin Sync:** Actively publishes `JointState` and `MarkerArray` data to mirror the physical arm, cubes, and bins in RViz in real-time.
* **Smart Trajectory Routing:** Adapts kinematic paths based on object location, utilizing a "Safe Tucked Pose" to prevent collisions when grabbing objects near the base.
* **Shape & Color Processing:** Uses HSV masking and contour analysis to distinguish solid cubes from hollow bins and calculates precise gripper rotation angles.
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

Flask-based stream with real-time detection
<img width="1920" height="1080" alt="Screenshot from 2026-04-11 12-15-59" src="https://github.com/user-attachments/assets/95182468-26b8-4fd4-acce-a9615d18bc48" />

Marker-based cube tracking in ROS 2
<img width="1920" height="1080" alt="Screenshot from 2026-04-11 12-19-00" src="https://github.com/user-attachments/assets/6d567ac8-de86-4077-a658-716004ba6ee2" />


## 🔮 Future Scope
1. **Voice-Controlled Manipulation:** Integrating NLP to trigger pick-and-place actions via voice commands.
2. **Cloud-Based Remote Digital Twin:** Enabling remote control and monitoring over a WAN using secure tunneling.
3. **Autonomous Defect Inspection:** Adding a quality control step where the AI inspects the object for damage before placement.
4. **Dynamic Target Interception:** Upgrading the vision pipeline with predictive kinematics to accurately track and pick moving objects, such as items on a motorized conveyor belt.
5. **Multi-Arm Collaboration:** Expanding the ROS 2 architecture to synchronize a secondary robot arm, allowing them to hand off objects or sort collaboratively within a shared workspace without         collisions.
