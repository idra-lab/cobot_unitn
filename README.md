# Instructions to setup the workspace (Raspberry Cobot)

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
## Cobot M5
This code can also be used for the cobot M5. However some small changes are needed:
- every occurrence of the serial port `AMA0` must be changed to `ACM0`. Changes must be done both in this package and in the packages supplied by the manufacturer;
- also the baudrate `1000000` has to be changed to `115200`.
These modifications can be made easily with VSCode search functionality.

The setup of the environment can be automatized using the shell script provided:
```bash 
chmod +x setup_m5.sh
./setup_m5.sh
```
Then remember to activate the `RoboCobot` environment with `conda activate /media/Dati/RoboCobot`.