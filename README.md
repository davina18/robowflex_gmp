PART 3: Robot Manipulation (Hands-on Session)
Robowflex Demos
Open a new terminal:
1. $ mkdir -p ~/cm3118_ws/src
Skip this step if you already have the workspace.
2. $ cd ~/cm3118_ws/src/
3. $ git clone https://github.com/sasilva1998/robowflex.git
Robowflex ROS package.
4. $ git clone https://github.com/KavrakiLab/robowflex_resources.git
ROS package with the description of several robots.
5. $ cd ~/cm3118_ws
6. $ catkin build -j 4
If you get and error about disk quota being exceeded, clean ROS logs using:
$ rosclean purge
7. $ source ~/.bashrc


# Installation Instructions

Create a workspace with Robowflex inside:
```sh
mkdir -p ~/rb_ws/src
cd rb_ws
git clone https://github.com/CardiffUniversityComputationalRobotics/robowflex.git
git clone https://github.com/CardiffUniversityComputationalRobotics/robowflex_resources.git
cd ~/rb_ws
catkin build
source ~/.bashrc
```

# Robowflex Motion Planning Demo

The following demo attempts a set of grasp poses in Robowflex:

In terminal 1 run:

```bash
roscore
```

In terminal 2 run:

```bash
rosrun robowflex_library panda_shelf
```

In terminal 3 run:

```bash
rosrun rviz rviz -d rb_ws/src/robowflex_gmp/robowflex_library/rviz/default1.rviz
```

:warning: **Warning** :warning: The paths in the following files must be changed to your own paths: panda_shelf.cpp.
