## 🔌 Hardware Connection & Wiring (MyCobot 280 Pi)

The main control software runs directly on the Raspberry Pi built into the base of the MyCobot. Before running any setup scripts, you must access the Pi's terminal.

### 1. Accessing the Robot
Choose one of the following methods to access the Raspberry Pi:

* **Method A: Direct Hardware Connection (Recommended)**
  1. Connect an HDMI monitor, USB mouse, and USB keyboard to the ports on the MyCobot base.
  2. Plug in the robot's power supply.
  3. Wait for the desktop to boot, then open the **Terminal** application.

* **Method B: Headless Access (SSH via Wi-Fi)**
  1. Ensure your laptop and the robot are connected to the exact same Wi-Fi network.
  2. Open the terminal on your personal laptop.
  3. Run the following command (replace with your robot's actual IP):
     `ssh pi@<ROBOT_IP_ADDRESS>`

### 2. Gripper Servo Wiring
To ensure the gripper opens and closes correctly, connect the 3-wire servo motor to the GPIO pins exactly as follows:

* 🟡 **Yellow Wire (Signal):** Connect to your PWM GPIO pin (e.g., **GPIO 4** or **GPIO 18**). Sends control signals to the servo.
* 🔴 **Red Wire (Power):** Connect to a **5V Pin** (Pin 2 or 4). Supplies the necessary power.
* 🟤 **Brown Wire (Ground):** Connect to any **GND Pin** (e.g., Pin 6). Completes the common ground connection.

> ⚠️ **Warning:** Always verify GPIO pins based on your specific configuration before powering on.
