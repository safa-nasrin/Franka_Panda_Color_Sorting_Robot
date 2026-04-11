from pymycobot.mycobot import MyCobot
import RPi.GPIO as GPIO
import time

# =========================================================
# 🎯 YOUR TRUE 6-AXIS COORDINATES 🎯
# =========================================================
RED_CUBE_6D = [250.4, 53.6, 124.8, 167.6, -10.6, 42.9]
RED_CONT_6D = [170.3, -100.9, 166.3, -177.8, 0.9, 44.7]

BLUE_CUBE_6D = [256.2, -100.2, 110.8, 158.3, -17.3, 48.5]
BLUE_CONT_6D = [168.2, 45.9, 164.1, -176.6, -7.0, 47.1]

HOVER_HEIGHT = 230.0
# =========================================================

# --- 1. SMOOTH SERVO SETUP ---
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(3, GPIO.OUT)
pwm = GPIO.PWM(3, 50)
pwm.start(0)

current_servo_angle = 0 

def control_servo_smooth(action):
    global current_servo_angle
    target_angle = 180 if action == "OPEN" else 0
    step = 5 if target_angle > current_servo_angle else -5
   
    if target_angle != current_servo_angle:
       for angle in range(current_servo_angle, target_angle + step, step):
           duty = angle / 18.0 + 2.5
           pwm.ChangeDutyCycle(duty)
           time.sleep(0.04)
       current_servo_angle = target_angle
    if action == "OPEN":
       pwm.ChangeDutyCycle(7.5)
    else:
       pwm.ChangeDutyCycle(2.5) 
    
    time.sleep(1)

# --- 2. ROBOT SETUP ---
mc = MyCobot('/dev/ttyAMA0', 1000000)
time.sleep(0.5)
mc.set_color(0, 0, 0) # Turn off LED
time.sleep(0.5)
ROBOT_SPEED = 25 

def manual_pick_and_place_6d(cube_6d, cont_6d, color):
    print(f"\n---> Moving {color} Cube to {color} Container <---")
    
    # Create hover coordinates by copying the taught positions but changing Z to HOVER_HEIGHT
    hover_above_cube = [cube_6d[0], cube_6d[1], HOVER_HEIGHT, cube_6d[3], cube_6d[4], cube_6d[5]]
    hover_above_cont = [cont_6d[0], cont_6d[1], HOVER_HEIGHT, cont_6d[3], cont_6d[4], cont_6d[5]]
    
    # 1. Open gripper 
    control_servo_smooth("OPEN")
   
    # 2. Hover high above the cube
    mc.send_coords(hover_above_cube,ROBOT_SPEED,0)
    time.sleep(3.5) 
    # 3. Drop straight down exactly as you taught it
    mc.send_coords(cube_6d, 15, 1) 
    time.sleep(5)
    
    # 4. Grab
    control_servo_smooth("CLOSE")
    
    # 5. Lift straight back up
    mc.send_coords(hover_above_cube, 25, 1)
    time.sleep(2)
    
    # 6. Move high above the container
    mc.send_coords(hover_above_cont, ROBOT_SPEED, 0)
    time.sleep(4)
    

    # 8. Release
    control_servo_smooth("OPEN")
    
    # 9. Lift straight up to avoid hitting container walls
    mc.send_coords(hover_above_cont, 15, 1)
    time.sleep(3)
    
    # 10. Reset home
    mc.send_coords([150, 0, 250, -180, 0, 0], ROBOT_SPEED, 0)
    control_servo_smooth("CLOSE") 
    time.sleep(2)

print("\n🚀 --- STARTING 6-AXIS TRUE COORDINATE TEST --- 🚀")

# Execute Red
manual_pick_and_place_6d(RED_CUBE_6D, RED_CONT_6D, "RED")

# Execute Blue
manual_pick_and_place_6d(BLUE_CUBE_6D, BLUE_CONT_6D, "BLUE")

print("\n✅ --- TEST COMPLETE --- ✅")
