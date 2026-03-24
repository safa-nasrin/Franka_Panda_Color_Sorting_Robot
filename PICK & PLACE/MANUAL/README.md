in manual folder - readme.md
## 🚀 Core Implementation: Phase 1 - Simulation & Digital Twin
Before executing AI vision commands, we established a bidirectional simulation environment using ROS 2 and RViz to bridge the gap between virtual and physical hardware.

### A. Forward Kinematics (Screen → Robot)
* Control Method: Allows direct control of the physical robot from the workstation.
* Mechanism: Adjusting individual joint angles via the Joint State Publisher GUI sliders in RViz.
* Real-time Sync: The physical MyCobot arm receives kinematics data and moves to the exact corresponding position instantly.

Commands to run Forward Kinematics:
source /opt/ros/foxy/setup.bash
cd ~/colcon_ws
colcon build
source install/setup.bash
ros2 launch mycobot_280pi slider_control.launch.py

### B. Digital Twin (Robot → Screen)
* Control Method: Reverse of Forward Kinematics; physical manipulation updates the digital model.
* Mechanism: Grasping the real robot arm and moving its joints by hand sends live telemetry data back to the PC.
* Real-time Sync: The virtual 3D model inside RViz immediately mirrors the hardware state.

Commands to run the Digital Twin:
source ~/colcon_ws/install/setup.bash
ros2 launch mycobot_280pi mycobot_follow.launch.py

---

## 🦾 Core Implementation: Phase 2 - Manual 6D Precision Control

What is Manual Pick and Place?
* Pre-validation step: Proves the robot can physically reach and grip objects before implementing AI.
* Hardcoded Coordinates: Sets exact spatial coordinates (X, Y, Z) directly into the script for reliability.
* Blind Movement: Allows the robot to move to pre-taught positions without needing sensor feedback.
* Visual Verification: Utilizes a live camera feed option to verify gripper alignment before execution.

### Gripper Calibration & Hover Height
* Hardware Testing: Conducted isolated tests on the custom servo gripper to determine PWM duty cycle limits.
* Calibrated Values: Identified exact values for operation: Open = 7.5 and Close = 2.5.
* Motor Protection: Prevents motor burnout by avoiding over-extension of the servo.
* Safe Trajectory: Implemented a safe HOVER_Z parameter to force elevation before any horizontal transit.
* Collision Avoidance: Guarantees high-clearance movement to avoid knocking over objects in the workspace.

Gripper Test Script (gripper_test.py):
import RPi.GPIO as GPIO
import time
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(3, GPIO.OUT)
pwm = GPIO.PWM(3, 50)
pwm.start(0)
try:
    print("Testing Gripper - OPEN")
    pwm.ChangeDutyCycle(7.5)
    time.sleep(2)
    print("Testing Gripper - CLOSE")
    pwm.ChangeDutyCycle(2.5) 
    time.sleep(2)
finally:
    pwm.stop()
    GPIO.cleanup()

### Script Optimization
* Manual Calibration: Utilizes a highly optimized script (manual_pick.py) to calibrate the physical workspace.
* 6-Axis Control: Uses True 6-Axis Coordinates (X, Y, Z, RX, RY, RZ) via the send_coords API.
* Linear Mode: Employs linear movement mode (1) to ensure straight vertical lifts.
* Arc Bypass: Bypasses default joint-interpolation arcs that often cause hardware dragging.

To Run Manual Control:
Everything is done directly in the terminal.
# To view or edit the hardcoded coordinates:
nano manual_pick.py
# To execute the manual pick and place routine:
python3 manual_pick.py

---
