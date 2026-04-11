import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray
from pymycobot.mycobot import MyCobot
import RPi.GPIO as GPIO
import cv2
import numpy as np
import time
import threading
import math
from flask import Flask, Response, send_file

# ==========================================
# --- FLASK SERVER SETUP ----
# ==========================================
app = Flask(__name__)
global_frame = None

@app.route('/')
def index():
    try:
        return send_file('index2.html')
    except Exception as e:
        return f"<h1>Error: index.html not found!</h1><p>{e}</p>"

def generate_video():
    global global_frame
    while True:
        if global_frame is not None:
            ret, jpeg = cv2.imencode('.jpg', global_frame)
            if ret:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')
        time.sleep(0.05)

@app.route('/video_feed')
def video_feed():
    return Response(generate_video(), mimetype='multipart/x-mixed-replace; boundary=frame')

def run_flask():
    app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)

threading.Thread(target=run_flask, daemon=True).start()


# ==========================================
# --- NEOFLUX ROBOT BRAIN ---
# ==========================================
class NeoFluxBrain(Node):
    def __init__(self):
        super().__init__('neoflux_brain')
        self.serial_lock = threading.Lock()

        # ALL state variables defined first
        self.is_busy        = False
        self.stop_request   = False
        self.red_cube       = None
        self.red_cont       = None
        self.blue_cube      = None
        self.blue_cont      = None
        self.camera_running = True
        
        # 🧠 MEMORY VARIABLES
        self.held_color  = None  
        self.red_placed  = False 
        self.blue_placed = False 
        self.gripper_open = True

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

        # Servo Setup (Pin 4)
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(4, GPIO.OUT)
        self.pwm = GPIO.PWM(4, 50)
        self.pwm.start(0)

        # ROS Topics
        self.subscription = self.create_subscription(
            String, '/web_arm_command', self.command_callback, 10)
        self.joint_pub  = self.create_publisher(JointState,  '/joint_states',   10)
        self.marker_pub = self.create_publisher(MarkerArray, '/object_markers',  10)

        # Timers
        self.create_timer(0.2, self.sync_digital_twin)
        self.create_timer(0.2, self.publish_markers)
        self.create_timer(5.0, self._keep_screen_off)

        # ── THE PERFECT PERSPECTIVE MATRIX ──
        pts_px = np.array([
            [17,  282], # Point 1: Top-Left
            [197, 291], # Point 2: Top-Right
            [205, 102], # Point 3: Bottom-Right
            [20,  102]  # Point 4: Bottom-Left
        ], dtype="float32")

        pts_robot = np.array([
            [274.9, -113.9], # Point 1: Top-Left
            [292.1,   63.4], # Point 2: Top-Right
            [125.5,   79.0], # Point 3: Bottom-Right
            [112.2, -115.5]  # Point 4: Bottom-Left
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
            if angles and len(angles) == 6:
                msg = JointState()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.name = [
                    'joint2_to_joint1', 'joint3_to_joint2',
                    'joint4_to_joint3', 'joint5_to_joint4',
                    'joint6_to_joint5', 'joint6output_to_joint6',
                    'gripper_left_joint', 'gripper_right_joint'
                ]
                gripper_pos =0.015 if self.gripper_open else 0.0
                msg.position = [math.radians(float(a)) for a in angles] + [gripper_pos, gripper_pos]
                self.joint_pub.publish(msg)
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

            # 4 vertical corner pillars
            markers.append(pillar(0,  cx+w/2, cy+w/2, h/2, t, t, h))
            markers.append(pillar(1,  cx+w/2, cy-w/2, h/2, t, t, h))
            markers.append(pillar(2,  cx-w/2, cy+w/2, h/2, t, t, h))
            markers.append(pillar(3,  cx-w/2, cy-w/2, h/2, t, t, h))

            # 4 bottom edges
            markers.append(pillar(4,  cx,     cy+w/2, t/2, w, t, t))
            markers.append(pillar(5,  cx,     cy-w/2, t/2, w, t, t))
            markers.append(pillar(6,  cx+w/2, cy,     t/2, t, w, t))
            markers.append(pillar(7,  cx-w/2, cy,     t/2, t, w, t))

            # 4 top edges
            markers.append(pillar(8,  cx,     cy+w/2, h,   w, t, t))
            markers.append(pillar(9,  cx,     cy-w/2, h,   w, t, t))
            markers.append(pillar(10, cx+w/2, cy,     h,   t, w, t))
            markers.append(pillar(11, cx-w/2, cy,     h,   t, w, t))

            # BOTTOM PLATE
            markers.append(pillar(12, cx, cy, t/2, w, w, t))

            return markers

        # 🟥 RED CUBE LOGIC (Always ID 1)
        if self.held_color == "RED":
            marker_array.markers.extend(make_solid_cube(1, 0, 15, 0.20, 1.0, 0.0, 0.0, "joint6"))
        elif self.red_placed and self.red_cont:
            px, py = self.pixel_to_mm(self.red_cont[0], self.red_cont[1], is_cube=False)
            marker_array.markers.extend(make_solid_cube(1, px, py, 0.025, 1.0, 0.0, 0.0))
        elif self.red_cube and not self.red_placed:
            x, y = self.pixel_to_mm(self.red_cube[0], self.red_cube[1])
            marker_array.markers.extend(make_solid_cube(1, x, y, 0.0125, 1.0, 0.0, 0.0))

        # 🟥 RED CONTAINER LOGIC (Always IDs 10 to 22)
        if self.red_cont:
            x, y = self.pixel_to_mm(self.red_cont[0], self.red_cont[1], is_cube=False)
            marker_array.markers.extend(make_open_container(10, x, y, 1.0, 0.4, 0.4))

        # 🟦 BLUE CUBE LOGIC (Always ID 2)
        if self.held_color == "BLUE":
            marker_array.markers.extend(make_solid_cube(2, 0, 0, 0.15, 0.0, 0.3, 1.0, "joint6"))
        elif self.blue_placed and self.blue_cont:
            px, py = self.pixel_to_mm(self.blue_cont[0], self.blue_cont[1], is_cube=False)
            marker_array.markers.extend(make_solid_cube(2, px, py, 0.025, 0.0, 0.3, 1.0))
        elif self.blue_cube and not self.blue_placed:
            x, y = self.pixel_to_mm(self.blue_cube[0], self.blue_cube[1])
            marker_array.markers.extend(make_solid_cube(2, x, y, 0.0125, 0.0, 0.3, 1.0))

        # 🟦 BLUE CONTAINER LOGIC (Always IDs 30 to 42)
        if self.blue_cont:
            x, y = self.pixel_to_mm(self.blue_cont[0], self.blue_cont[1], is_cube=False)
            marker_array.markers.extend(make_open_container(30, x, y, 0.3, 0.6, 1.0))

        self.marker_pub.publish(marker_array) 
   
    # ── GRIPPER ──────────────────────────────────────────────────────────────
    def control_servo_smooth(self, action: str):
        target_duty = 4.0 if action == "OPEN" else 9.0
        self.gripper_open = (action == "OPEN")
        self.pwm.ChangeDutyCycle(target_duty)
        time.sleep(0.5)
        self.pwm.ChangeDutyCycle(0)

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

                self.red_cube,  self.red_cont  = self.detect_and_draw(frame, mask_red,  "Red")
                self.blue_cube, self.blue_cont = self.detect_and_draw(frame, mask_blue, "Blue")

                global_frame = frame
            time.sleep(0.03)
        cap.release()

    # ── CLEAN PIXEL TO MM ────────────────────────────────────────────────────
    def pixel_to_mm(self, px, py, is_cube=True):
        pt = np.array([[[px, py]]], dtype="float32")
        transformed = cv2.perspectiveTransform(pt, self.transform_matrix)
        x, y = transformed[0][0]
        # Look how clean this is! No manual offsets needed because your matrix is perfect!
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

        # 🧠 SMART ROUTING: Safe Tucked Poses for Bottom Zone
        if tx < 150.0: 
            self.get_logger().info("Cube is in the Bottom Zone! Using Safe Tucked Pose.")
            
            with self.serial_lock:
                if ty < -30.0:
                    # BOTTOM LEFT POSE
                    self.mc.send_angles([-19.42, -15.73, -140.8, 65.74, -6.67, -167.6], 25)
                elif ty > 30.0:
                    # BOTTOM RIGHT POSE
                    self.mc.send_angles([45.17, -15.64, -140.8, 63.19, 14.85, -100.1], 25)
                else:
                    # BOTTOM CENTER POSE
                    self.mc.send_angles([7.2, -16.17, -140.8, 64.59, 3.25, -132.89], 25)
            
            # Wait for the arm to fold into the safe position
            if not wait(3.5): self.is_busy = False; return
            
            # Now drop straight down to grab!
            with self.serial_lock:
                self.mc.send_coords([tx, ty, 110, rx, ry, target_rz], 15, 1)
            if not wait(3.5): self.is_busy = False; return

        else:
            # 🟢 NORMAL ROUTING: For cubes far away (Top Row)
            with self.serial_lock:
                self.mc.send_coords([tx, ty, 230, rx, ry, target_rz], 25, 0)
            if not wait(3.5): self.is_busy = False; return

            with self.serial_lock:
                self.mc.send_coords([tx, ty, 110, rx, ry, target_rz], 15, 1)
            if not wait(3.5): self.is_busy = False; return

        # 🛑 GRAB CUBE
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

        # 🛑 RELEASE CUBE
        self.control_servo_smooth("OPEN")
        self.held_color = None
        time.sleep(0.8)

        with self.serial_lock:
            self.mc.send_coords([px, py, 230, rx, ry, base_rz], 25, 0)
        if not wait(2.0): self.is_busy = False; return

        self.control_servo_smooth("CLOSE")
        self.is_busy = False
        
        # 🛑 LOCK IN MEMORY
        if color == "RED": self.red_placed = True
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
            self.red_placed = False # 🧠 Reset memory
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
            self.blue_placed = False # 🧠 Reset memory
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


def main(args=None):
    rclpy.init(args=args)
    brain = NeoFluxBrain()
    try:
        rclpy.spin(brain)
    except Exception:
        pass
    finally:
        brain.camera_running = False
        GPIO.cleanup()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
