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

### 2. Gripper Servo Wiring (External Battery Setup)
To ensure the gripper opens and closes correctly without overloading the Raspberry Pi, connect the 3-wire servo motor using a breadboard and an external battery:

* 🟡 **Yellow Wire (Signal):** Connect to your PWM GPIO pin (e.g., **GPIO 4** or **GPIO 18**). Sends control signals to the servo.
* 🔴 **Red Wire (Power):** Connect to the positive terminal of the **6V Li-Poly RC Battery**.
* 🟤 **Brown Wire (Ground):** Connect to the common ground rail on the **breadboard**. *(Note: You must also connect a Raspberry Pi GND pin and the battery's negative terminal to this same breadboard rail to complete the circuit).*

> ⚠️ **Warning:** Do NOT try to connect more than 6V to the servo, as it will cause permanent damage. Never connect the 6V battery directly to the Raspberry Pi pins.
