"""
=============================================================================
Project: NeoFlux AI - LLM-Powered Cobot Pick-and-Place

Description:
This module acts as a "System-Aware" Senior Robotics Engineer. It bridges a 
conversational AI chatbot (Llama-3.1/Groq) with physical hardware, featuring:
 - Real-time RViz Digital Twin synchronization
 - Live telemetry monitoring and physical pick-and-place execution
 - Natural language processing for dynamic hardware control and safety
=============================================================================
"""

  
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray
from pymycobot.mycobot import MyCobot
import serial # Added for Arduino Vacuum Gripper
import cv2
import numpy as np
import time
import threading
import math
import datetime
import requests
from flask import Flask, Response, jsonify, request, send_from_directory
from flask_cors import CORS

# ==========================================
# --- GROQ API SETUP ---
# ==========================================
GROQ_API_KEY = "YOUR_GROQ_API_KEY"
GROQ_URL     = "https://api.groq.com/openai/v1/chat/completions"
GROQ_MODEL   = "llama-3.1-8b-instant"

robotics_prompt = """
You are the NeoFlux System Assistant, a Senior Robotics & Electronics Engineer.
Your job is to provide highly accurate, detailed, and expert-level advice.

CRITICAL RULES:
1. MEMORY: You have a conversational memory. Reply naturally to follow-up questions.
2. BE SMART & CRITICAL: Warn the user if they suggest dangerous electrical connections.
3. SCANNABLE FORMATTING: Use plain text, bullet points (-), and short paragraphs. NEVER use HTML tags
4. CONTEXT: You specialize in ROS 2, OpenCV, Arduino Serial, MyCobot arms, and pneumatics.
"""

chat_history = []
print("✅ GROQ AI BRIDGE: ONLINE (Llama-3.1)")

# ==========================================
# --- FLASK & REST API SETUP ---
# ==========================================
app = Flask(__name__, static_folder=".")
CORS(app)
global_frame = None


# ==========================================
# --- NEOFLUX ROBOT BRAIN ---
# ==========================================
class NeoFluxBrain(Node):
    def __init__(self):
        super().__init__('neoflux_brain')
        self.serial_lock = threading.Lock()

        # State variables
        self.is_busy        = False
        self.stop_request   = False
        self.red_cube       = None
        self.red_cont       = None
        self.blue_cube      = None
        self.blue_cont      = None
        self.camera_running = True

        # Memory variables
        self.held_color  = None
        self.red_placed  = False
        self.blue_placed = False
        self.gripper_open = True

        # Telemetry storage for AI dashboard
        self.current_angles = {}
        self.current_coords = {"X": 0.0, "Y": 0.0, "Z": 0.0}
        self._start_time = datetime.datetime.now(datetime.timezone.utc)

        # Connect to Robot
        self.get_logger().info('Booting Robot & Initiating Blackout...')
        with self.serial_lock:
            self.mc = MyCobot('/dev/ttyAMA0', 1000000)
            self.mc.power_on()
            time.sleep(2.0)
            try:
                self.mc.set_color(0, 0, 0)
                time.sleep(0.1)
                self.mc.set_color(0, 0, 0)
            except Exception:
                pass

        # ================================================
        # Arduino Vacuum Setup (Replaced GPIO Servo)
        # ================================================
        self.arduino_port = '/dev/ttyACM0'
        self.arduino_baud = 9600
        try:
            self.gripper_serial = serial.Serial(self.arduino_port, self.arduino_baud, timeout=1)
            time.sleep(2)  # allow Arduino reset
            self.get_logger().info("Serial connected to Arduino Vacuum Pump")
        except Exception as e:
            self.get_logger().error(f"Failed to connect Arduino Vacuum: {e}")
            self.gripper_serial = None

        # ROS Topics
        self.subscription = self.create_subscription(
            String, '/web_arm_command', self.command_callback, 10)
        self.joint_pub  = self.create_publisher(JointState,  '/joint_states',  10)
        self.marker_pub = self.create_publisher(MarkerArray, '/object_markers', 10)

        # Timers
        self.create_timer(0.2, self.sync_digital_twin)
        self.create_timer(0.2, self.publish_markers)
        self.create_timer(5.0, self._keep_screen_off)

        # Perspective Transform Matrix
        pts_px = np.array([
            [17,  282],
            [197, 291],
            [205, 102],
            [20,  102]
        ], dtype="float32")

        pts_robot = np.array([
            [274.9, -113.9],
            [292.1,  63.4],
            [125.5,  79.0],
            [112.2, -115.5]
        ], dtype="float32")

        self.transform_matrix = cv2.getPerspectiveTransform(pts_px, pts_robot)

        threading.Thread(target=self.camera_loop, daemon=True).start()
        self.get_logger().info('NeoFlux Brain: Ready — Screen Blacked Out, Vision Active.')

    # ── KEEP SCREEN OFF ──────────────────────────────────────────────────────
    def _keep_screen_off(self):
        if self.is_busy:
            return
        if self.serial_lock.acquire(blocking=False):
            try:
                self.mc.set_color(0, 0, 0)
            except Exception:
                pass
            finally:
                self.serial_lock.release()

    # ── DIGITAL TWIN SYNC ────────────────────────────────────────────────────
    def sync_digital_twin(self):
        if not self.serial_lock.acquire(blocking=False):
            return
        try:
            angles = self.mc.get_angles()
            coords = self.mc.get_coords()
            if angles and len(angles) == 6:
                msg = JointState()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.name = [
                    'joint2_to_joint1', 'joint3_to_joint2',
                    'joint4_to_joint3', 'joint5_to_joint4',
                    'joint6_to_joint5', 'joint6output_to_joint6',
                    'gripper_left_joint', 'gripper_right_joint'
                ]
                gripper_pos = 0.015 if self.gripper_open else 0.0
                msg.position = [math.radians(float(a)) for a in angles] + [gripper_pos, gripper_pos]
                self.joint_pub.publish(msg)
                self.current_angles = {f"J{i+1}": round(angles[i], 2) for i in range(6)}

            if coords and len(coords) >= 3:
                self.current_coords = {
                    "X": round(coords[0], 2),
                    "Y": round(coords[1], 2),
                    "Z": round(coords[2], 2)
                }
        except Exception:
            pass
        finally:
            self.serial_lock.release()

    # ── PUBLISH MARKERS ──────────────────────────────────────────────────────
    def publish_markers(self):
        marker_array = MarkerArray()

        def make_solid_cube(m_id, x_mm, y_mm, z_m, r, g, b, frame="joint1"):
            m = Marker()
            m.header.frame_id = frame
            m.header.stamp.sec = 0
            m.header.stamp.nanosec = 0
            m.ns = "objects"
            m.id = m_id
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.scale.x = m.scale.y = m.scale.z = 0.025
            m.pose.position.x = x_mm / 1000.0
            m.pose.position.y = y_mm / 1000.0
            m.pose.position.z = z_m
            m.pose.orientation.w = 1.0
            m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 1.0
            m.lifetime.sec = 1
            return [m]

        def make_open_container(base_id, x_mm, y_mm, r, g, b):
            markers = []
            cx = x_mm / 1000.0
            cy = y_mm / 1000.0
            w = 0.06
            h = 0.04
            t = 0.004

            def pillar(idx, px, py, pz, sx, sy, sz):
                m = Marker()
                m.header.frame_id = "joint1"
                m.header.stamp.sec = 0
                m.header.stamp.nanosec = 0
                m.ns = "objects"
                m.id = base_id + idx
                m.type = Marker.CUBE
                m.action = Marker.ADD
                m.pose.position.x = px
                m.pose.position.y = py
                m.pose.position.z = pz
                m.pose.orientation.w = 1.0
                m.scale.x = sx
                m.scale.y = sy
                m.scale.z = sz
                m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 1.0
                m.lifetime.sec = 1
                return m

            markers.append(pillar(0,  cx+w/2, cy+w/2, h/2, t, t, h))
            markers.append(pillar(1,  cx+w/2, cy-w/2, h/2, t, t, h))
            markers.append(pillar(2,  cx-w/2, cy+w/2, h/2, t, t, h))
            markers.append(pillar(3,  cx-w/2, cy-w/2, h/2, t, t, h))
            markers.append(pillar(4,  cx,     cy+w/2, t/2, w, t, t))
            markers.append(pillar(5,  cx,     cy-w/2, t/2, w, t, t))
            markers.append(pillar(6,  cx+w/2, cy,     t/2, t, w, t))
            markers.append(pillar(7,  cx-w/2, cy,     t/2, t, w, t))
            markers.append(pillar(8,  cx,     cy+w/2, h,   w, t, t))
            markers.append(pillar(9,  cx,     cy-w/2, h,   w, t, t))
            markers.append(pillar(10, cx+w/2, cy,     h,   t, w, t))
            markers.append(pillar(11, cx-w/2, cy,     h,   t, w, t))
            markers.append(pillar(12, cx,     cy,     t/2, w, w, t))
            return markers

        # Red cube
        if self.held_color == "RED":
            marker_array.markers.extend(make_solid_cube(1, 0, 0, 0.12, 1.0, 0.0, 0.0, "joint6"))
        elif self.red_placed and self.red_cont:
            px, py = self.pixel_to_mm(self.red_cont[0], self.red_cont[1], is_cube=False)
            marker_array.markers.extend(make_solid_cube(1, px, py, 0.0125, 1.0, 0.0, 0.0))
        elif self.red_cube and not self.red_placed:
            x, y = self.pixel_to_mm(self.red_cube[0], self.red_cube[1])
            marker_array.markers.extend(make_solid_cube(1, x, y, 0.0125, 1.0, 0.0, 0.0))

        # Red container
        if self.red_cont:
            x, y = self.pixel_to_mm(self.red_cont[0], self.red_cont[1], is_cube=False)
            marker_array.markers.extend(make_open_container(10, x, y, 1.0, 0.4, 0.4))

        # Blue cube
        if self.held_color == "BLUE":
            marker_array.markers.extend(make_solid_cube(2, 0, 0, 0.14, 0.0, 0.3, 1.0, "joint6"))
        elif self.blue_placed and self.blue_cont:
            px, py = self.pixel_to_mm(self.blue_cont[0], self.blue_cont[1], is_cube=False)
            marker_array.markers.extend(make_solid_cube(2, px, py, 0.0125, 0.0, 0.3, 1.0))
        elif self.blue_cube and not self.blue_placed:
            x, y = self.pixel_to_mm(self.blue_cube[0], self.blue_cube[1])
            marker_array.markers.extend(make_solid_cube(2, x, y, 0.0125, 0.0, 0.3, 1.0))

        # Blue container
        if self.blue_cont:
            x, y = self.pixel_to_mm(self.blue_cont[0], self.blue_cont[1], is_cube=False)
            marker_array.markers.extend(make_open_container(30, x, y, 0.3, 0.6, 1.0))

        self.marker_pub.publish(marker_array)

    # ── GRIPPER (Arduino Serial Vacuum Pump) ─────────────────────────────────
    def control_servo_smooth(self, action: str):
        self.gripper_open = (action == "OPEN")
        
        if self.gripper_serial:
            if action == "OPEN":
                self.gripper_serial.write(b'r')   # Release
                self.get_logger().info("Vacuum Pump: RELEASE (r)")
            else:
                self.gripper_serial.write(b's')   # Suction (Close)
                self.get_logger().info("Vacuum Pump: SUCTION (s)")
        
        # Delay allows the vacuum time to build suction or drop the object
        time.sleep(0.5)

    # ── DETECTION ────────────────────────────────────────────────────────────
    def detect_and_draw(self, frame, mask, color_name):
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cube_data = cont_coords = None
        best_cube_area = best_cont_area = 0
        box_color = (0, 0, 255) if color_name == "Red" else (255, 0, 0)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 300: continue
            x, y, w, h = cv2.boundingRect(cnt)
            if not (0.3 < w / float(h) < 3.5): continue
            bbox_area = w * h

            if bbox_area > 4000:
                if bbox_area > best_cont_area:
                    best_cont_area = bbox_area
                    cont_coords = (x + w // 2, y + h // 2)
                cv2.rectangle(frame, (x, y), (x + w, y + h), box_color, 2)
                cv2.putText(frame, f"{color_name} Container",
                            (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, box_color, 2)
                cv2.circle(frame, (x + w // 2, y + h // 2), 4, (255, 255, 255), -1)

            elif 600 <= bbox_area <= 4000:
                hull_area = cv2.contourArea(cv2.convexHull(cnt))
                if (area / hull_area if hull_area > 0 else 0) < 0.60: continue
                if bbox_area > best_cube_area:
                    best_cube_area = bbox_area
                    rect = cv2.minAreaRect(cnt)
                    (center_x, center_y), (rect_w, rect_h), angle = rect
                    if rect_w < rect_h:
                        angle += 90
                    angle = angle % 90
                    if angle > 45:
                        angle -= 90
                    cube_data = (x + w // 2, y + int(h * 0.85), angle)
                    box = cv2.boxPoints(rect)
                    box = np.int0(box)
                    cv2.drawContours(frame, [box], 0, box_color, 2)
                    cv2.putText(frame, f"{color_name} Cube {int(angle)}deg",
                                (int(center_x) - 30, int(center_y) - 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, box_color, 2)
                    cv2.circle(frame, (int(center_x), int(center_y)), 4, (255, 255, 255), -1)

        return cube_data, cont_coords

    # ── CAMERA LOOP ──────────────────────────────────────────────────────────
    def camera_loop(self):
        global global_frame
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  320)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

        while self.camera_running:
            ret, frame = cap.read()
            if ret:
                frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
                h, w = frame.shape[:2]
                frame = cv2.warpAffine(frame,
                    cv2.getRotationMatrix2D((w // 2, h // 2), 6.5, 1.0), (w, h))
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

                mask_red = (
                    cv2.inRange(hsv, np.array([0,   130, 60]), np.array([10,  255, 255]))
                  + cv2.inRange(hsv, np.array([165, 130, 60]), np.array([180, 255, 255]))
                )
                mask_blue = cv2.inRange(
                    hsv, np.array([100, 140, 60]), np.array([130, 255, 255]))

                kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
                mask_red  = cv2.dilate(cv2.morphologyEx(
                    mask_red,  cv2.MORPH_OPEN, kernel), None, iterations=2)
                mask_blue = cv2.dilate(cv2.morphologyEx(
                    mask_blue, cv2.MORPH_OPEN, kernel), None, iterations=2)

                # 1. Detect objects from the camera frame
                r_cube, r_cont = self.detect_and_draw(frame, mask_red,  "Red")
                b_cube, b_cont = self.detect_and_draw(frame, mask_blue, "Blue")

                # 2. SMART VISION LOCK
                if not self.is_busy:
                    # Cubes always update normally
                    self.red_cube  = r_cube
                    self.blue_cube = b_cube

                    # SMART CONTAINER MEMORY:
                    # If a cube was placed, do NOT let the container disappear!
                    if self.red_placed and r_cont is None:
                        pass # The camera lost it, but keep the old memory!
                    else:
                        self.red_cont = r_cont # Update normally

                    if self.blue_placed and b_cont is None:
                        pass # The camera lost it, but keep the old memory!
                    else:
                        self.blue_cont = b_cont # Update normally
                global_frame = frame
            time.sleep(0.03)
        cap.release()

    # ── PIXEL TO MM ──────────────────────────────────────────────────────────
    def pixel_to_mm(self, px, py, is_cube=True):
        pt = np.array([[[px, py]]], dtype="float32")
        transformed = cv2.perspectiveTransform(pt, self.transform_matrix)
        x, y = transformed[0][0]
        return round(x, 1), round(y, 1)

    # ── PICK & PLACE ─────────────────────────────────────────────────────────
    def auto_pick_and_place(self, tx, ty, px, py, color, cube_angle):
        self.is_busy = True
        self.stop_request = False

        def wait(t):
            start = time.time()
            while time.time() - start < t:
                if self.stop_request:
                    return False
                time.sleep(0.05)
            return True

        if color == "RED":
            rx, ry, base_rz = 167.6, -10.6, 42.9
        else:
            rx, ry, base_rz = 158.3, -17.3, 48.5

        target_rz = base_rz + cube_angle

        self.get_logger().info(
            f"PICKING {color}: cube=({tx},{ty}) cont=({px},{py}) angle={int(cube_angle)}deg")

        self.control_servo_smooth("OPEN")

        if tx < 150.0:
            self.get_logger().info("Cube is in the Bottom Zone! Using Safe Tucked Pose.")
            with self.serial_lock:
                if ty < -30.0:
                    self.mc.send_angles([-19.42, -15.73, -140.8, 65.74, -6.67, -167.6], 25)
                elif ty > 30.0:
                    self.mc.send_angles([45.17, -15.64, -140.8, 63.19, 14.85, -100.1], 25)
                else:
                    self.mc.send_angles([7.2, -16.17, -140.8, 64.59, 3.25, -132.89], 25)
            if not wait(3.5): self.is_busy = False; return
            with self.serial_lock:
                self.mc.send_coords([tx, ty, 110, rx, ry, target_rz], 15, 1)
            if not wait(3.5): self.is_busy = False; return
        else:
            with self.serial_lock:
                self.mc.send_coords([tx, ty, 230, rx, ry, target_rz], 25, 0)
            if not wait(3.5): self.is_busy = False; return
            with self.serial_lock:
                self.mc.send_coords([tx, ty, 110, rx, ry, target_rz], 15, 1)
            if not wait(3.5): self.is_busy = False; return

        self.control_servo_smooth("CLOSE")
        self.held_color = color
        time.sleep(0.5)

        with self.serial_lock:
            self.mc.send_coords([tx, ty, 230, rx, ry, target_rz], 25, 1)
        if not wait(3.0): self.is_busy = False; return

        with self.serial_lock:
            self.mc.send_coords([px, py, 230, rx, ry, base_rz], 25, 0)
        if not wait(3.5): self.is_busy = False; return

        with self.serial_lock:
            self.mc.send_coords([px, py, 180, rx, ry, base_rz], 20, 0)
        if not wait(2.0): self.is_busy = False; return

        self.control_servo_smooth("OPEN")
        self.held_color = None
        time.sleep(0.8)

        with self.serial_lock:
            self.mc.send_coords([px, py, 230, rx, ry, base_rz], 25, 0)
        if not wait(2.0): self.is_busy = False; return

        self.control_servo_smooth("CLOSE")
        self.is_busy = False

        if color == "RED":  self.red_placed  = True
        if color == "BLUE": self.blue_placed = True

        self.get_logger().info(f"DONE: {color} cube placed in container")

    # ── COMMAND CALLBACK ─────────────────────────────────────────────────────
    def command_callback(self, msg):
        cmd = msg.data.upper()

        if cmd == 'STOP':
            self.stop_request = True
            with self.serial_lock:
                self.mc.stop()
            self.is_busy = False
            return

        if self.is_busy and cmd != 'UNLOCK':
            return

        if cmd == 'PICK RED':
            # 🧹 WIPE THE SLATE CLEAN: Delete old Blue ghosts so they don't overlap!
            self.blue_placed = False
            self.blue_cube = None
            self.blue_cont = None

            self.red_placed = False
            if self.red_cube and self.red_cont:
                cx, cy, angle = self.red_cube
                tx, ty = self.pixel_to_mm(cx, cy, is_cube=True)
                px, py = self.pixel_to_mm(self.red_cont[0], self.red_cont[1], is_cube=False)
                threading.Thread(
                    target=self.auto_pick_and_place,
                    args=(tx, ty, px, py, "RED", angle)).start()
            else:
                self.get_logger().warn("PICK RED: cube or container not detected!")

        elif cmd == 'PICK BLUE':
            # 🧹 WIPE THE SLATE CLEAN: Delete old Red ghosts so they don't overlap!
            self.red_placed = False
            self.red_cube = None
            self.red_cont = None

            self.blue_placed = False
            if self.blue_cube and self.blue_cont:
                cx, cy, angle = self.blue_cube
                tx, ty = self.pixel_to_mm(cx, cy, is_cube=True)
                px, py = self.pixel_to_mm(self.blue_cont[0], self.blue_cont[1], is_cube=False)
                threading.Thread(
                    target=self.auto_pick_and_place,
                    args=(tx, ty, px, py, "BLUE", angle)).start()
            else:
                self.get_logger().warn("PICK BLUE: cube or container not detected!")
        elif cmd == 'HOME':
            with self.serial_lock:
                self.mc.send_angles([0, 0, 0, 0, 0, 0], 25)

        elif cmd == 'UNLOCK':
            with self.serial_lock:
                self.mc.release_all_servos()
            self.is_busy = False

        elif cmd == 'LOCK':
            with self.serial_lock:
                self.mc.power_on()
            self.is_busy = False


# ==========================================
# --- HELPERS ---
# ==========================================
def _uptime_str(brain_node):
    delta = datetime.datetime.now(datetime.timezone.utc) - brain_node._start_time
    h, rem = divmod(int(delta.total_seconds()), 3600)
    return f"{h:02d}h {divmod(rem, 60)[0]:02d}m"


# ==========================================
# --- FLASK ROUTES ---
# ==========================================
@app.route("/")
def index():
    return send_from_directory(".", "index3.html")


@app.route("/chat", methods=["POST"])
def chat():
    global brain_node, chat_history

    body = request.get_json(silent=True)
    if not body or "message" not in body:
        return jsonify({"error": "Body must be JSON with a 'message' field."}), 400

    original_msg = str(body["message"]).strip()
    user_msg = original_msg.lower()

    if not original_msg:
        return jsonify({"error": "The 'message' field must not be empty."}), 400

    # ── Fast local responses (Telemetry formatted beautifully for HTML) ─────────
    # ── Fast local responses (Plain Text Formatting) ─────────
    if "status" in user_msg:
        hw    = "CONNECTED" if brain_node.mc else "OFFLINE"
        grip  = "OPEN" if brain_node.gripper_open else "CLOSED"
        reply = (
            f"📊 SYSTEM TELEMETRY\n"
            f"- Hardware: {hw}\n"
            f"- Uptime: {_uptime_str(brain_node)}\n"
            f"- Gripper: {grip}\n"
            f"- Busy: {'YES' if brain_node.is_busy else 'NO'}"
        )
        return jsonify({"reply": reply})

    elif "coord" in user_msg:
        c     = brain_node.current_coords
        reply = (
            f"📍 REAL-TIME COORDINATES\n"
            f"- X: {c['X']} mm\n"
            f"- Y: {c['Y']} mm\n"
            f"- Z: {c['Z']} mm"
        )
        return jsonify({"reply": reply})

    elif "joint" in user_msg:
        ja    = brain_node.current_angles
        lines = "\n".join([f"- {k} : {v}°" for k, v in ja.items()])
        reply = f"🦾 JOINT CONFIGURATION\n{lines}"
        return jsonify({"reply": reply})
    # ── Groq AI for everything else ──────────────────────────────────────────
    try:
        # Build message list (OpenAI format that Groq uses)
        messages = [{"role": "system", "content": robotics_prompt}]

        # Include last 10 exchanges so the AI has memory
        for entry in chat_history[-10:]:
            messages.append(entry)

        # Add the current user message
        messages.append({"role": "user", "content": original_msg})

        payload = {
            "model":       GROQ_MODEL,
            "messages":    messages,
            "max_tokens":  512,
            "temperature": 0.7
        }

        headers = {
            "Authorization": f"Bearer {GROQ_API_KEY}",
            "Content-Type":  "application/json"
        }

        response = requests.post(GROQ_URL, json=payload, headers=headers, timeout=15)
        data     = response.json()

        if "choices" in data:
            reply_text = data["choices"][0]["message"]["content"]

            # Save to history so memory works across turns
            chat_history.append({"role": "user", "content": original_msg})
            chat_history.append({"role": "assistant", "content": reply_text})

            # Keep history from growing forever (last 20 messages)
            if len(chat_history) > 20:
                chat_history = chat_history[-20:]

            reply = f"<strong>🤖 NEOFLUX AI:</strong><br><br>{reply_text}"

        elif data.get("error", {}).get("code") == 429:
            print(f"\n⚠️  GROQ RATE LIMIT HIT: {data}\n")
            reply = "⚠️ AI rate limit reached. Please wait a moment and try again."

        else:
            print(f"\n❌ GROQ API ERROR: {data}\n")
            reply = "😔 AI Error. Check the terminal for details."

    except requests.exceptions.Timeout:
        print("\n❌ GROQ TIMEOUT: Request took too long\n")
        reply = "😔 AI request timed out. Check your internet connection."

    except Exception as e:
        print(f"\n❌ NETWORK ERROR: {e}\n")
        reply = "😔 Network connection to AI failed."

    return jsonify({"reply": reply})


@app.route('/video_feed')
def video_feed():
    def gen():
        while True:
            if global_frame is not None:
                _, jpeg = cv2.imencode('.jpg', global_frame)
                yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'
                       + jpeg.tobytes() + b'\r\n')
            time.sleep(0.05)
    return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')


# ==========================================
# --- MAIN ---
# ==========================================
def main(args=None):
    global brain_node
    rclpy.init(args=args)
    brain_node = NeoFluxBrain()

    # Start Flask in background thread
    threading.Thread(
        target=lambda: app.run(
            host='0.0.0.0', port=5001, debug=False, use_reloader=False),
        daemon=True
    ).start()

    try:
        rclpy.spin(brain_node)
    finally:
        brain_node.camera_running = False
        # Cleanly shut down the Arduino serial port so it doesn't stay stuck suctioning
        if brain_node.gripper_serial:
            brain_node.gripper_serial.write(b'r')  # ensure safe release
            brain_node.gripper_serial.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
           
       
                    
      


           
  
