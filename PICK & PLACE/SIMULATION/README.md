# MyCobot 280 Digital Twin & AI Vision Automation

## 📖 Project Overview
A vision-guided manipulation system where objects are detected using a camera, localized using pose estimation, and picked using a cobot arm in both simulation and real hardware. Integrates ROS 2, OpenCV, kinematics, and real-time robot control to simulate industrial automation workflows.

## ⚙️ Environment Setup & Installation
**OS & ROS Version:** Ubuntu 20.04 | ROS 2 Foxy

### 1. Install ROS 2 System Dependencies
If this is your first time setting up the machine, ensure the base ROS 2 tools are installed:
```bash
sudo apt update && sudo apt upgrade
sudo apt install ros-foxy-desktop ros-foxy-moveit ros-foxy-joint-state-publisher-gui
```

### 2. 🚀 Quick Start (Automated Setup)
To instantly configure the Python virtual environment, install hardware dependencies (`pymycobot`, `opencv-python`, `numpy`, etc.), build the workspace, and launch the project, simply run the master script:

```bash
chmod +x run_project.sh
./run_project.sh
```

> **🛠️ Developer Troubleshooting Note (Package Names):** > During initial setup, standard launch commands may fail if the custom environment package is uniquely named (e.g., `firefighter`) instead of the default. If you are building this manually, verify your exact package name by listing the contents of your source folder: `ls ~/colcon_ws/src`

---

## 🤖 Core Implementation: Phase 1 - Simulation & Digital Twin
Before executing AI vision commands, we established a bidirectional simulation environment using ROS 2 and RViz to bridge the gap between virtual and physical hardware.

### A. Forward Kinematics (Screen → Robot)
*(Select Option 1 in the Quick Start Menu)*

This process allows us to control the physical robot directly from the computer. By adjusting the individual joint angles using the sliders inside the **Joint State Publisher GUI** window in RViz, the physical MyCobot arm receives the kinematics data and moves to the exact corresponding position in real-time.
![joint_state_publisher](https://github.com/user-attachments/assets/3019955c-a0d5-422b-9d98-73ab36bac575)
![forward kinematics](https://github.com/user-attachments/assets/ad4a2ac1-b64e-439f-b332-8b689f714170)

### B. Digital Twin (Robot → Screen)
*(Select Option 2 in the Quick Start Menu)*

This is the reverse of Forward Kinematics. When we physically grasp the real robot arm and move its joints by hand, the arm sends live telemetry data back to the computer. The virtual 3D model inside RViz immediately mirrors those exact movements, creating a perfect real-time digital twin of the hardware state.
