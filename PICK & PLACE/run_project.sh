#!/bin/bash

echo "🤖 Starting Project Setup & Virtual Environment..."

# 1. Create and activate virtual environment
python3 -m venv myenv
source myenv/bin/activate
echo "Installing dependencies..."
pip install -r requirements.txt

# 2. Setup ROS 2 Environment
echo "⚙️ Building ROS 2 Workspace..."
source /opt/ros/foxy/setup.bash

# Navigate to your workspace
cd ~/colcon_ws 
colcon build
source install/setup.bash

# 3. Interactive Menu 
echo ""
echo "========================================"
echo "🚀 SETUP COMPLETE! WHICH MODULE TO RUN?"
echo "========================================"
echo "--- PHASE 1: SIMULATION ---"
echo "1) Forward Kinematics (Slider Control)"
echo "2) Digital Twin (MyCobot Follow)"
echo "--- PHASE 2: HARDWARE & AI ---"
echo "3) Manual Pick & Place"
echo "4) Real-Time AI Pick & Place (Web & Simulation)"
echo "========================================"
read -p "Enter 1, 2, 3, or 4: " choice

if [ "$choice" == "1" ]; then
    echo "Launching Forward Kinematics..."
    ros2 launch mycobot_280pi slider_control.launch.py
elif [ "$choice" == "2" ]; then
    echo "Launching Digital Twin..."
    ros2 launch mycobot_280pi mycobot_follow.launch.py
elif [ "$choice" == "3" ]; then
    echo "Launching Manual Pick & Place..."
    python3 src/MANUAL_PNP/pick_place.py 
elif [ "$choice" == "4" ]; then
    echo "Launching Real-Time AI Pick & Place (Multi-Process)..."
    
    # MAGIC TRICK: This ensures that when you press Ctrl+C, it kills ALL background processes safely!
    trap 'echo "🛑 Stopping all processes..."; kill 0' SIGINT

    # Process 1 (Terminal 1): Rosbridge Server
    echo "--> [1/3] Starting rosbridge_server..."
    ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
    sleep 3 # Give the server 3 seconds to boot up cleanly

    # Process 2 (Terminal 3): Build and Launch Digital Twin
    # Note: We put this inside parentheses () so the 'cd' command doesn't mess up the rest of the script
    echo "--> [2/3] Building and starting Digital Twin..."
    (cd ~/colcon_ws && colcon build --packages-select mycobot_280pi mycobot_description && source install/setup.bash && ros2 launch ~/colcon_ws/digital_twin.launch.py) &
    sleep 5 # Give ROS 2 and RViz 5 seconds to fully load up

    # Process 3 (Terminal 2): Python Web Script
    # Notice this one DOES NOT have an '&'. It runs in the foreground so you can see its output and interact with it!
    echo "--> [3/3] Starting Python AI script..."
    python3 src/REALTIME_PNP/pick_place.py
    
    # Wait prevents the script from closing until you press Ctrl+C
    wait
else
    echo "Invalid choice. Exiting script."
fi
