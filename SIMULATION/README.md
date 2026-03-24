# MyCobot 280 Digital Twin & AI Vision Automation

## 📖 Project Overview
A vision-guided manipulation system where objects are detected using a camera, localized using pose estimation, and picked using a cobot arm in both simulation and real hardware. Integrates ROS 2, OpenCV, kinematics, and real-time robot control to simulate industrial automation workflows.

---

## ⚙️ Environment Setup & Installation
**OS & ROS Version:** Ubuntu 20.04 - ROS 2 Foxy

**1. Install ROS 2 Foxy, MoveIt 2, and RViz GUIs**
```bash
sudo apt update && sudo apt upgrade
sudo apt install ros-foxy-desktop
sudo apt install ros-foxy-moveit
sudo apt install ros-foxy-joint-state-publisher-gui
```

**2. Install Python Hardware Dependencies**
```bash
pip3 install pymycobot opencv-python numpy flask RPi.GPIO
```

> **🛠️ Developer Troubleshooting Note (Finding Your Package Name):** > During the initial setup, the standard simulation launch commands failed because the custom environment package was uniquely named `firefighter` instead of the default name. 
> If you are building this, you can verify your exact package name by listing the contents of your source folder with this command:
> ```bash
> ls ~/colcon_ws/src
> ```

---

## 🚀 Core Implementation: Phase 1 - Simulation & Digital Twin
Before executing AI vision commands, we established a bidirectional simulation environment using ROS 2 and RViz to bridge the gap between virtual and physical hardware.

### A. Forward Kinematics (Screen → Robot)
This process allows us to control the physical robot directly from the computer. By adjusting the individual joint angles using the sliders inside the **Joint State Publisher GUI** window in RViz, the physical MyCobot arm receives the kinematics data and moves to the exact corresponding position in real-time.

**Commands to run Forward Kinematics:**
```bash
source /opt/ros/foxy/setup.bash
cd ~/colcon_ws
colcon build
source install/setup.bash
ros2 launch mycobot_280pi slider_control.launch.py
```

### B. Digital Twin (Robot → Screen)
This is the reverse of Forward Kinematics. When we physically grasp the real robot arm and move its joints by hand, the arm sends live telemetry data back to the computer. The virtual 3D model inside RViz immediately mirrors those exact movements, creating a perfect real-time digital twin of the hardware state.

**Commands to run the Digital Twin:**
```bash
# 1. Navigate to workspace
cd ~/colcon_ws

# 2. Build ONLY if the 'install' folder is missing or you changed the code
colcon build 

# 3. Source the setup file (MUST do this in every new terminal)
source install/setup.bash

# 4. Launch the digital twin
ros2 launch mycobot_280pi mycobot_follow.launch.py
```

---
