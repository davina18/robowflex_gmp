# Installation Instructions

```sh
git clone https://github.com/CardiffUniversityComputationalRobotics/robowflex_gmp
cd ~/robowflex_gmp
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
rosrun rviz rviz -d robowflex_gmp/src/robowflex_gmp/robowflex_library/rviz/default1.rviz
```

:warning: **Warning** :warning: The paths in the following files must be changed to your own paths: panda_shelf.cpp.
