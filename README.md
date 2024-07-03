# Installation Instructions

Create a workspace with Robowflex inside:```sh
cd ~
mkdir -p rb_ws/src
cd rb_ws
source /opt/ros/melodic/setup.bash # if you haven't already
catkin config --init
cd src
git clone https://github.com/KavrakiLab/robowflex.git
catkin build
```

To try out a demo script, you first need a robot description.
The easiest to try is the _Fetch_ robot, either by debian or source:
```sh
# Debian
sudo apt install ros-melodic-fetch-ros

# Or, Source
cd ~/rb_ws/src
git clone https://github.com/fetchrobotics/fetch_ros
catkin build
```

After the workspace is built, source and run a demo:
```sh
cd ~/rb_ws
source ./devel/setup.bash
rosrun robowflex_library fetch_test
Navigate to the following directory:

```bash
cd test_franka_gmp_simulation_bringup/src/
```
