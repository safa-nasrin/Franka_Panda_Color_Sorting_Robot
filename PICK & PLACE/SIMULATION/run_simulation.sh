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

# Navigate to your workspace (Make sure this matches your cloned repo folder!)
cd ~/colcon_ws 
colcon build
source install/setup.bash

# 3. Interactive Menu 
echo ""
echo "========================================"
echo "🚀 SETUP COMPLETE! WHICH MODULE TO RUN?"
echo "========================================"
echo "1) Forward Kinematics (Slider Control)"
echo "2) Digital Twin (MyCobot Follow)"
echo "========================================"
read -p "Enter 1 or 2: " choice

if [ "$choice" == "1" ]; then
    echo "Launching Forward Kinematics..."
    ros2 launch mycobot_280pi slider_control.launch.py
elif [ "$choice" == "2" ]; then
    echo "Launching Digital Twin..."
    ros2 launch mycobot_280pi mycobot_follow.launch.py
else
    echo "Invalid choice. Exiting script."
fi
