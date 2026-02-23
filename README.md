# Instructions to setup the workspace

## Create the folder of the workspace
```bash
cd
mkdir -p ws_cobot/src
cd ~/ws_cobot/src
```

## Get all the packages supplied by the constructor
```bash
git clone https://github.com/elephantrobotics/mycobot_ros2.git
```
## Add cobot_unitn, the package contained in this repo
```bash
cd  mycobot_ros2/mycobot_280/
git clone https://github.com/idra-lab/cobot_unitn
```
## Build the packages
```bash
cd ../../..
colcon build --symlink-install
```
## Source the workspace
```bash
source install/local_setup.bash
```
## Now it is possible to run the code of this repo
```bash
ros2 launch cobot_unitn trajectory.launch.py
```
