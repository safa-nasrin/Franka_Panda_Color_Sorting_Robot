"""
// Hardware Calibration: Manually move the robot to 4 reference points on the table.
// Captures real-world robot X and Y coordinates to sync with camera pixel data.
"""

from pymycobot.mycobot import MyCobot
import time

# 1. Connect to the robot
print("Connecting to robot...")
mc = MyCobot('/dev/ttyAMA0', 1000000)
time.sleep(1)

# 2. Unlock the servos so you can move it by hand
print("Unlocking servos...")
mc.release_all_servos()
time.sleep(1)

print("\n✅ Robot unlocked! You can now move the arm by hand.")
print("Move the gripper to your 4 tape marks one by one.")
print("-" * 50)

point_count = 1
while point_count <= 4:
    input(f"Move the arm to Tape #{point_count} and press ENTER...")
    
    # Get the coordinates from the robot
    coords = mc.get_coords()
    
    if coords and len(coords) >= 2:
        x = coords[0]
        y = coords[1]
        print(f"📍 Tape #{point_count} Robot Coordinates -> [ {x}, {y} ]\n")
        point_count += 1
    else:
        print("⚠️ Couldn't read coordinates, try pressing ENTER again.\n")

print("🎉 DONE! Write these 4 points down and put them in your main code.")
