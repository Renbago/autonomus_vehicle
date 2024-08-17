**Hi to everyone! i just wanted to say you can easily run this project buy using docker branch instead of creating locally.**

# Project explanation

The autonomous driving  and connectivity algorithms on 1/10 scale vehicles, provided by the company, to navigate in a designated environment simulating a miniature smart city. This project designed for the competation named as BOSCH FUTURE MOBILITY CHALLANGE. 


# Whats included ?
I also wanted to share The foundation of this code was inspired by and further developed upon the work in the following GitHub repositories:

This project using [Non-linear MPC with CasADi](https://github.com/MMehrez/MPC-and-MHE-implementation-in-MATLAB-using-Casadi/tree/master/workshop_github), [Dynamic/static obstacle detection](https://github.com/jk-ethz/obstacle_detector), [robot_localization](https://github.com/cra-ros-pkg/robot_localization), Rviz visualization, djikstra and [yolo](https://github.com/ultralytics/ultralytics). 

## The simulation and real life outputs from project

 [![KOU-Mekatronom Youtube Channel](https://youtube.com/playlist?list=PLDE_vDxu0Gkk-s3ndTqIScKTHSvL8dt0m&si=bbp9Qc9xVI-1Tctj)](https://youtube.com/playlist?list=PLDE_vDxu0Gkk-s3ndTqIScKTHSvL8dt0m&si=bbp9Qc9xVI-1Tctj) 

# Dependencies
This project has been writed for [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) version

The casadi installation process;
```
cd /home/<USERNAME>
sudo apt-get install -y \
 gfortran \
 coinor-libipopt-dev \
 
git clone --recurse-submodules https://github.com/casadi/casadi.git
mkdir -p casadi/build && cd build && cmake -DWITH_IPOPT=true .. && sudo make install && sudo apt update

```

## Prerequisites
By rosdep firstly we should install dependencis.

```
sudo apt-get update
sudo apt-get install python3-rosdep
sudo rosdep init
rosdep update
```
### Step 1 

```cd /path/to/your/ros/workspace ``` and ```rosdep install --from-paths src --ignore-src -r -y```

### Step2
```
current_dir=$(/path/to/your/ros/workspace)

echo 'export ROS_PACKAGE_PATH="'$current_dir'/src:$ROS_PACKAGE_PATH"' >> ~/.bashrc
echo 'export GAZEBO_MODEL_PATH="'$current_dir'/src/models_pkg:$GAZEBO_MODEL_PATH"' >> ~/.bashrc

source ~/.bashrc
```
or
```
echo 'export ROS_PACKAGE_PATH="/home/<USERNAME>/<worksapcename>/src:$ROS_PACKAGE_PATH"' >> ~/.bashrc
echo 'export GAZEBO_MODEL_PATH="/home/<USERNAME>/<worksapcename>/src/models_pkg:$GAZEBO_MODEL_PATH"' >> ~/.bashrc
 
source ~/.bashrc
```

# Outputs from Gazebo

<img src="https://github.com/ECC-BFMC/Simulator/blob/main/Picture1.png" width=30% height=30%>

# From new parkour:

<img src="https://github.com/KOU-Mekatronom/Simulator/blob/main/parkour.png" width=30% height=30%>

# From added RVIZ:

<img src="https://github.com/KOU-Mekatronom/Simulator/blob/main/rviz.png" width=30% height=30%>

# Traffic lights plugin:

<img src="https://github.com/KOU-Mekatronom/Simulator/blob/main/traffic_lights_pkg.gif" width=30% height =30%>

# BFMC Simulator Project

The project contains the entire Gazebo simulator. 
- It can also be used to get a better overview of how the competition is environment looks like
- It can be used in to develop the vehicle state machine
- It can be used to simulate the path planning
- It can be used to set-up a configuration procedure for the real track
- Not suggested for image processing
- Try not to fall in the "continuous simulator developing" trap

Tips on how to install and work on it, can be found in the 

## The documentation is available in details here:
[Documentation](https://bosch-future-mobility-challenge-documentation.readthedocs-hosted.com/data/simulator.html)

This project includes the algorithms has been made from KOU-Mekatronom::
- It has robot_localization package, you can fuse the gps and IMU data easily.
- Robot_localization package config path is ```src/example/config/ekf_localization.yaml```
- Added urdf and lidar.sdf 
- It has laser_scan now topic name is ```/automobile/scan``` for bostacle_detection.
- Added TF2 package the tf tree visualization ```frames.pdf``` 
- Added traffic lights publisher, ```src/sim_pkg/launch/sublaunchers/traffic_lights.launch```
- In your main code you need to subscribe ``` automobile/trafficlight/master,slave,start topics```

# Contributors
[Mehmet Baha Dursun](https://github.com/Renbago)
