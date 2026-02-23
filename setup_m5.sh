#!/bin/bash

# -----------------------------
# Start ROS workspace and clone repos
# -----------------------------
source /opt/ros/humble/setup.bash
echo "Creating workspace..."
cd /media/Dati
mkdir -p ws_cobot_m5/src
cd ws_cobot_m5/src

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
# Build the workspace
# -----------------------------

cd ws_cobot_m5
echo "Building workspace with colcon..."
colcon build --symlink-install

# -----------------------------
# Create Python virtual environment and install packages
# -----------------------------

cd ..
echo "Creating and activating Python virtual environment RoboCobot..."
source /usr/LDSS/bin/conda.sh
conda create --prefix ./RoboCobot
conda activate ./RoboCobot
conda install pip
pip install scipy
pip install pymycobot

echo "Done!"
