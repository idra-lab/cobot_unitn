#!/bin/bash

# -----------------------------
# Start ROS workspace and clone repos
# -----------------------------
StartRos
echo "Creating workspace..."
cd /media/Dati
mkdir -p ws_cobot/src
cd ws_cobot/src

# Clone main mycobot ROS2 repository
echo "Cloning mycobot_ros2..."
git clone https://github.com/elephantrobotics/mycobot_ros2.git

# Clone your cobot_unitn repo inside the mycobot_280 folder
cd mycobot_ros2/mycobot_280/
echo "Cloning cobot_unitn..."
git clone https://github.com/idra-lab/cobot_unitn

# -----------------------------
# Apply text replacements in Python files
# -----------------------------
cd ../../..  # Go back to root

# Replace AMA0 with ACM0 in all Python files
echo "Replacing AMA0 with ACM0..."
find . -type f -name "*.py" -exec sed -i 's/AMA0/ACM0/g' {} +

# Replace 1000000 with 115200 in all Python files
echo "Replacing 1000000 with 115200..."
find . -type f -name "*.py" -exec sed -i 's/1000000/115200/g' {} +

# -----------------------------
# Create Python virtual environment and install packages
# -----------------------------

cd ..
echo "Creating Python virtual environment RoboCobot..."
python3 -m venv RoboCobot

echo "Activating virtual environment..."
source RoboCobot/bin/activate

echo "Upgrading pip..."
pip install --upgrade pip

echo "Installing required Python packages..."
pip install pymycobot scipy numpy

# -----------------------------
# Build the workspace
# -----------------------------

cd ws_cobot
echo "Building workspace with colcon..."
colcon build --symlink-install

echo "Done!"