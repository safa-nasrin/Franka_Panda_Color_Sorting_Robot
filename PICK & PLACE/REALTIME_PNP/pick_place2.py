import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState, Image
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
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
        self.is_busy        = False
        self.stop_request   = False
        self.red_cube       = None
        self.red_cont       = None
        self.blue_cube      = None
        self.blue_cont      = None
        self.camera_running = True
        
        # 🎯 DIGITAL TWIN STATES
        self.red_state = "TABLE"
        self.blue_state = "TABLE"
        self.wrist_frame = "gripper_base" # Custom URDF Gripper

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
        self.joint_pub  = self.create_publisher(JointState,  '/joint_states',   10)
        self.marker_pub = self.create_publisher(MarkerArray, '/object_markers',  10)
        
        # RViz Live Camera Publisher
        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, '/camera/live_feed', 10)

        # Timers
        self.create_timer(0.2, self.sync_digital_twin)
        self.create_timer(0.2, self.publish_markers)
        self.create_timer(5.0, self._keep_screen_off)

        # 🎯 YOUR BRAND NEW, PERFECT CALIBRATION MATRICES 🎯
        pts_px = np.array([
            [17,  282],
            [197, 291],
            [205, 102],
            [20,  102]
        ], dtype="float32")

        pts_robot = np.array([
            [274.9, -113.9],
            [292.1,   63.4],
            [125.5,   79.0],
            [112.2, -115.5]
        ], dtype="float32")

        self.transform_matrix = cv2.getPerspectiveTransform(pts_px, pts_robot)

        threading.Thread(target=self.camera_loop, daemon=True).start()
        self.get_logger().info('NeoFlux Brain: Ready — Fresh Calibration + No Offsets!')

    # ── KEEP SCREEN OFF ──────────────────────────────────────────────────────
    def _keep_screen_off(self):
        if self.is_busy: return 
        if self.serial_lock.acquire(blocking=False):
            try: self.mc.set_color(0, 0, 0)
            except Exception: pass
            finally: self.serial_lock.release()

    # ── DIGITAL TWIN SYNC ────────────────────────────────────────────────────
    def sync_digital_twin(self):
        if not self.serial_lock.acquire(blocking=False): return
        try:
            angles = self.mc.get_angles()
            if angles and len(angles) == 6:
                msg = JointState()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.name = ['joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3', 'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6']
                msg.position = [math.radians(float(a)) for a in angles]
                self.joint_pub.publish(msg)
        except Exception: pass
        finally: self.serial_lock.release()

    # ── PUBLISH MARKERS ──────────────────────────────────────────────────────
    def publish_markers(self):
        marker_array = MarkerArray()
        mid = 0

        # Function for the small solid cubes
        def make_solid_cube(m_id, x_mm, y_mm, r, g, b, state="TABLE"):
            m = Marker()
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "objects"; m.type = Marker.CUBE; m.action = Marker.ADD
            m.scale.x = m.scale.y = m.scale.z = 0.025
            m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 1.0
            m.lifetime.sec = 1

            if state == "HELD":
                m.id = m_id + 1000
                m.header.frame_id = self.wrist_frame
                m.pose.position.x = 0.0
                m.pose.position.y = 0.0
                m.pose.position.z = 0.04 # Between the custom gripper fingers
                m.pose.orientation.w = 1.0
            elif state == "BIN":
                m.id = m_id + 2000
                m.header.frame_id = "joint1"
                m.pose.position.x = x_mm / 1000.0
                m.pose.position.y = y_mm / 1000.0
                m.pose.position.z = 0.04
                m.pose.orientation.w = 1.0
            else: # TABLE
                m.id = m_id
                m.header.frame_id = "joint1"
                m.pose.position.x = x_mm / 1000.0
                m.pose.position.y = y_mm / 1000.0
                m.pose.position.z = 0.0125
                m.pose.orientation.w = 1.0
            return [m]

        # Function for transparent wireframe containers
        def make_open_container(base_id, x_mm, y_mm, r, g, b):
            markers = []
            cx = x_mm / 1000.0
            cy = y_mm / 1000.0
            w, h, t = 0.06, 0.04, 0.002
            
            def pillar(offset, px, py, pz, sx, sy, sz):
                m = Marker()
                m.header.frame_id = "joint1"
                m.header.stamp = self.get_clock().now().to_msg()
                m.ns = "objects"; m.id = base_id + offset
                m.type = Marker.CUBE; m.action = Marker.ADD
                m.pose.position.x = px; m.pose.position.y = py; m.pose.position.z = pz
                m.scale.x = sx; m.scale.y = sy; m.scale.z = sz
                m.pose.orientation.w = 1.0
                m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 0.2
                m.lifetime.sec = 1
                return m

            markers.append(pillar(0, cx+w/2, cy+w/2, h/2, t, t, h))
            markers.append(pillar(1, cx+w/2, cy-w/2, h/2, t, t, h))
            markers.append(pillar(2, cx-w/2, cy+w/2, h/2, t, t, h))
            markers.append(pillar(3, cx-w/2, cy-w/2, h/2, t, t, h))
            markers.append(pillar(4, cx, cy+w/2, t/2, w, t, t))
            markers.append(pillar(5, cx, cy-w/2, t/2, w, t, t))
            markers.append(pillar(6, cx+w/2, cy, t/2, t, w, t))
            markers.append(pillar(7, cx-w/2, cy, t/2, t, w, t))
            markers.append(pillar(8, cx, cy+w/2, h, w, t, t))
            markers.append(pillar(9, cx, cy-w/2, h, w, t, t))
            markers.append(pillar(10, cx+w/2, cy, h, t, w, t))
            markers.append(pillar(11, cx-w/2, cy, h, t, w, t))
            markers.append(pillar(12, cx, cy, t/2, w, w, t))        
            return markers

        # ── RED CUBE ──
        if self.red_state == "HELD":
            marker_array.markers.extend(make_solid_cube(mid, 0, 0, 1.0, 0.0, 0.0, "HELD"))
            mid += 10
        elif self.red_state == "BIN" and self.red_cont:
            x, y = self.pixel_to_mm(self.red_cont[0], self.red_cont[1])
            marker_array.markers.extend(make_solid_cube(mid, x, y, 1.0, 0.0, 0.0, "BIN"))
            mid += 10
        elif self.red_cube and self.red_state == "TABLE":
            x, y = self.pixel_to_mm(self.red_cube[0], self.red_cube[1])
            marker_array.markers.extend(make_solid_cube(mid, x, y, 1.0, 0.0, 0.0, "TABLE"))
            mid += 10

        # ── RED CONTAINER ──
        if self.red_cont:
            x, y = self.pixel_to_mm(self.red_cont[0], self.red_cont[1])
            marker_array.markers.extend(make_open_container(mid, x, y, 1.0, 0.4, 0.4))
            mid += 20

        # ── BLUE CUBE ──
        if self.blue_state == "HELD":
            marker_array.markers.extend(make_solid_cube(mid, 0, 0, 0.0, 0.3, 1.0, "HELD"))
            mid += 10
        elif self.blue_state == "BIN" and self.blue_cont:
            x, y = self.pixel_to_mm(self.blue_cont[0], self.blue_cont[1])
            marker_array.markers.extend(make_solid_cube(mid, x, y, 0.0, 0.3, 1.0, "BIN"))
            mid += 10
        elif self.blue_cube and self.blue_state == "TABLE":
            x, y = self.pixel_to_mm(self.blue_cube[0], self.blue_cube[1])
            marker_array.markers.extend(make_solid_cube(mid, x, y, 0.0, 0.3, 1.0, "TABLE"))
            mid += 10

        # ── BLUE CONTAINER ──
        if self.blue_cont:
            x, y = self.pixel_to_mm(self.blue_cont[0], self.blue_cont[1])
            marker_array.markers.extend(make_open_container(mid, x, y, 0.3, 0.6, 1.0))
            mid += 20

        self.marker_pub.publish(marker_array)

    # ── GRIPPER ──────────────────────────────────────────────────────────────
    def control_servo_smooth(self, action: str):
        target_duty = 4.0 if action == "OPEN" else 9.0
        
        try:
            # Send the signal using the proven standalone logic
            self.pwm.ChangeDutyCycle(target_duty)
            
            # Give it exactly 1 second to physically move
            time.sleep(1.0)
            
            # Cut the signal so the servo doesn't overload and freeze
            self.pwm.ChangeDutyCycle(0)
            
        except Exception as e:
            self.get_logger().error(f"Gripper Error: {e}")

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
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  320)
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
                    cv2.inRange(hsv, np.array([0,   130, 60]), np.array([10,  255, 255]))
                  + cv2.inRange(hsv, np.array([165, 130, 60]), np.array([180, 255, 255]))
                )
                mask_blue = cv2.inRange(
                    hsv, np.array([100, 140, 60]), np.array([130, 255, 255]))

                kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
                mask_red  = cv2.dilate(cv2.morphologyEx(
                    mask_red,  cv2.MORPH_OPEN, kernel), None, iterations=2)
                mask_blue = cv2.dilate(cv2.morphologyEx(
                    mask_blue, cv2.MORPH_OPEN, kernel), None, iterations=2)

                self.red_cube,  self.red_cont  = self.detect_and_draw(frame, mask_red,  "Red")
                self.blue_cube, self.blue_cont = self.detect_and_draw(frame, mask_blue, "Blue")

                global_frame = frame
                
                # 🎯 BROADCAST VIDEO TO RVIZ
                try:
                    img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                    self.image_pub.publish(img_msg)
                except Exception:
                    pass

            time.sleep(0.03)
        cap.release()

    # ── PIXEL TO MM ──────────────────────────────────────────────────────────
    def pixel_to_mm(self, px, py):
        pt = np.array([[[px, py]]], dtype="float32")
        transformed = cv2.perspectiveTransform(pt, self.transform_matrix)
        x, y = transformed[0][0]
        
        # 🎯 NO OFFSETS! Pure mathematically exact values.
        return round(x, 1), round(y, 1)

    # ── PICK & PLACE ─────────────────────────────────────────────────────────
    def auto_pick_and_place(self, tx, ty, px, py, color, cube_angle):
        self.is_busy = True
        self.stop_request = False

        def wait(t):
            start = time.time()
            while time.time() - start < t:
                if self.stop_request: return False
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

        with self.serial_lock:
            self.mc.send_coords([tx, ty, 230, rx, ry, target_rz], 25, 0)
        if not wait(3.5): self.is_busy = False; return

        with self.serial_lock:
            self.mc.send_coords([tx, ty, 110, rx, ry, target_rz], 15, 1)
        if not wait(3.5): self.is_busy = False; return

        self.control_servo_smooth("CLOSE")
        
        # 🎯 RVIZ STATE UPDATE
        if color == "RED": self.red_state = "HELD"
        else: self.blue_state = "HELD"

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
        
        # 🎯 RVIZ STATE UPDATE
        if color == "RED": self.red_state = "BIN"
        else: self.blue_state = "BIN"

        with self.serial_lock:
            self.mc.send_coords([px, py, 230, rx, ry, base_rz], 25, 0)
        if not wait(2.0): self.is_busy = False; return

        self.control_servo_smooth("CLOSE")
        self.is_busy = False
        self.get_logger().info(f"DONE: {color} cube placed in container")

    # ── COMMAND CALLBACK ─────────────────────────────────────────────────────
    def command_callback(self, msg):
        cmd = msg.data.upper().strip()

        # 🎯 RESET DIGITAL TWIN STATES
        if cmd in ['STOP', 'UNLOCK', 'HOME', 'RESET']:
            self.red_state = "TABLE"
            self.blue_state = "TABLE"

        if cmd == 'STOP':
            self.stop_request = True
            with self.serial_lock:
                self.mc.stop()
            self.is_busy = False
            return

        if self.is_busy and cmd != 'UNLOCK':
            return

        if cmd == 'PICK RED':
            self.red_state = "TABLE"
            if self.red_cube and self.red_cont:
                cx, cy, angle = self.red_cube
                tx, ty = self.pixel_to_mm(cx, cy)
                px, py = self.pixel_to_mm(self.red_cont[0], self.red_cont[1])
                threading.Thread(
                    target=self.auto_pick_and_place,
                    args=(tx, ty, px, py, "RED", angle)).start()
            else:
                self.get_logger().warn("PICK RED: cube or container not detected!")

        elif cmd == 'PICK BLUE':
            self.blue_state = "TABLE"
            if self.blue_cube and self.blue_cont:
                cx, cy, angle = self.blue_cube
                tx, ty = self.pixel_to_mm(cx, cy)
                px, py = self.pixel_to_mm(self.blue_cont[0], self.blue_cont[1])
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

