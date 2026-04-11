import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pymycobot.mycobot import MyCobot
import RPi.GPIO as GPIO
import cv2
import numpy as np
import time
import threading
from flask import Flask, Response

# --- FLASK SERVER SETUP ---
app = Flask(__name__)
global_frame = None

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

class NeoFluxBrain(Node):
    def __init__(self):
        super().__init__('neoflux_brain')
        self.mc = MyCobot('/dev/ttyAMA0', 1000000)
        self.mc.power_on()
        
        # Smooth Servo Setup (Pin D3)
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(3, GPIO.OUT)
        self.pwm = GPIO.PWM(3, 50)
        self.pwm.start(0)
        self.current_servo_angle = 0
        
        self.red_cube = None
        self.red_cont = None
        self.blue_cube = None
        self.blue_cont = None
        self.is_busy = False    
        
        self.subscription = self.create_subscription(String, '/web_arm_command', self.command_callback, 10)
        self.camera_running = True
        threading.Thread(target=self.camera_loop, daemon=True).start()
        self.get_logger().info('🟢 NeoFlux AI Brain: Ready with Exact Working Positions')

    # --- UPDATED GRIPPER LOGIC ---
    def control_servo_smooth(self, action: str):
        target_angle = 180 if action == "OPEN" else 0
        step = 5 if target_angle > self.current_servo_angle else -5
        
        if target_angle != self.current_servo_angle:
            for angle in range(self.current_servo_angle, target_angle + step, step):
                duty = angle / 18.0 + 2.5
                self.pwm.ChangeDutyCycle(duty)
                time.sleep(0.04)
            self.current_servo_angle = target_angle
            
        if action == "OPEN":
            self.pwm.ChangeDutyCycle(7.5)
        else:
            self.pwm.ChangeDutyCycle(2.5)  # ← The critical fix for grip
            
        time.sleep(1)

    # --- DETECTION ENGINE ---
    def detect_and_draw(self, frame, mask, color_name):
        height, width = frame.shape[:2]
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cube_coords, cont_coords = None, None
        
        for cnt in contours:
            x, y, w, h = cv2.boundingRect(cnt)
            center_x, center_y = x + (w // 2), y + (h // 2)
            if center_x < 40 or center_x > (width - 40) or center_y < 40 or center_y > (height - 40): continue 
            
            if (w*h) > 600:
                box_color = (0, 0, 255) if color_name == "Red" else (255, 0, 0)
                if (w*h) > 4000:
                    cont_coords = (center_x, center_y)
                    label = f"{color_name} Container"
                else:
                    cube_coords = (center_x, center_y)
                    label = f"{color_name} Cube"
                cv2.rectangle(frame, (x, y), (x + w, y + h), box_color, 2)
                cv2.putText(frame, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, box_color, 2)
                
        return cube_coords, cont_coords

    def camera_loop(self):
        global global_frame
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        
        while self.camera_running:
            ret, frame = cap.read()
            if ret:
                frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
                h, w = frame.shape[:2]
                frame = cv2.warpAffine(frame, cv2.getRotationMatrix2D((w//2, h//2), 6.5, 1.0), (w, h))
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                mask_red = cv2.dilate(cv2.inRange(hsv, np.array([0,100,50]), np.array([10,255,255])) + cv2.inRange(hsv, np.array([160,100,50]), np.array([180,255,255])), None, iterations=2)
                mask_blue = cv2.dilate(cv2.inRange(hsv, np.array([100,150,0]), np.array([140,255,255])), None, iterations=2)
                
                self.red_cube, self.red_cont = self.detect_and_draw(frame, mask_red, "Red")
                self.blue_cube, self.blue_cont = self.detect_and_draw(frame, mask_blue, "Blue")
                global_frame = frame
            time.sleep(0.03)
        cap.release()

    # --- EXACT MATH FROM YOUR WORKING CODE ---
    def pixel_to_mm(self, px, py, is_cube=False):
        robot_x_origin = 220
        robot_y_origin = 15
        pixel_scale = 0.80
        cam_origin_x = 160
        cam_origin_y = 120
        
        x = robot_x_origin + ((py - cam_origin_y) * pixel_scale)
        y = robot_y_origin + ((px - cam_origin_x) * pixel_scale)
        
        # The exact 15mm forward reach fix for the cubes
        if is_cube:
            x += 15.0  
            y += 0.0   
            
        return round(max(100.0, min(250.0, x)), 1), round(max(-200.0, min(200.0, y)), 1)

    def auto_pick_and_place(self, tx, ty, px, py, color):
        self.is_busy = True
        speed = 25
        rx, ry, rz = (167.6, -10.6, 42.9) if color == "RED" else (158.3, -17.3, 48.5)
        crz = 44.7 if color == "RED" else 47.1
        
        hover_z = 230.0
        # Lowered to 110.0 to match your successful code!
        pick_z = 110.0 

        self.get_logger().info(f"🤖 AUTO-PICKING: {color}")
        
        # 1. Hover
        self.control_servo_smooth("OPEN")
        self.mc.send_coords([tx, ty, hover_z, rx, ry, rz], speed, 0); time.sleep(3.5)
        
        # 2. Down
        self.mc.send_coords([tx, ty, pick_z, rx, ry, rz], 15, 1); time.sleep(4)
        
        # 3. Grab
        self.control_servo_smooth("CLOSE")
        
        # 4. Lift Straight Up
        self.mc.send_coords([tx, ty, hover_z, rx, ry, rz], 25, 1); time.sleep(3.2)
        
        # 5. Move to Container
        self.mc.send_coords([px, py, hover_z, -177, 0, crz], speed, 0); time.sleep(4)
        
        # 6. Drop and Finish
        self.control_servo_smooth("OPEN")
        self.mc.send_coords([150, 0, 250, -180, 0, 0], speed, 0)
        self.control_servo_smooth("CLOSE")
        self.is_busy = False

    def command_callback(self, msg):
        cmd = msg.data.upper()
        if self.is_busy and cmd != 'UNLOCK': return
        
        if cmd == 'PICK RED':
            if self.red_cube and self.red_cont:
                tx, ty = self.pixel_to_mm(self.red_cube[0], self.red_cube[1], is_cube=True)
                px, py = self.pixel_to_mm(self.red_cont[0], self.red_cont[1], is_cube=False)
                threading.Thread(target=self.auto_pick_and_place, args=(tx, ty, px, py, "RED")).start()
                
        elif cmd == 'PICK BLUE':
            if self.blue_cube and self.blue_cont:
                tx, ty = self.pixel_to_mm(self.blue_cube[0], self.blue_cube[1], is_cube=True)
                px, py = self.pixel_to_mm(self.blue_cont[0], self.blue_cont[1], is_cube=False)
                threading.Thread(target=self.auto_pick_and_place, args=(tx, ty, px, py, "BLUE")).start()
                
        elif cmd == 'HOME': self.mc.send_angles([0,0,0,0,0,0], 25)
        elif cmd == 'UNLOCK': self.mc.release_all_servos(); self.is_busy = False

def main(args=None):
    rclpy.init(args=args)
    brain = NeoFluxBrain()
    try: rclpy.spin(brain)
    except: pass
    finally:
        brain.camera_running = False; GPIO.cleanup(); brain.mc.release_all_servos(); rclpy.shutdown()

if __name__ == '__main__':
    main()
