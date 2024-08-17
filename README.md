# Project explanation

The autonomous driving  and connectivity algorithms on 1/10 scale vehicles, provided by the company, to navigate in a designated environment simulating a miniature smart city. This project designed for the competation named as BOSCH FUTURE MOBILITY CHALLANGE. 


# Whats included ?
I also wanted to share The foundation of this code was inspired by and further developed upon the work in the following GitHub repositories:

This project using [Non-linear MPC with CasADi](https://github.com/MMehrez/MPC-and-MHE-implementation-in-MATLAB-using-Casadi/tree/master/workshop_github), [Dynamic/static obstacle detection](https://github.com/jk-ethz/obstacle_detector), [robot_localization](https://github.com/cra-ros-pkg/robot_localization), Rviz visualization, djikstra and [yolo](https://github.com/ultralytics/ultralytics). 

## The simulation and real life outputs from project

 [![KOU-Mekatronom Youtube Channel](https://youtube.com/playlist?list=PLDE_vDxu0Gkk-s3ndTqIScKTHSvL8dt0m&si=bbp9Qc9xVI-1Tctj)](https://youtube.com/playlist?list=PLDE_vDxu0Gkk-s3ndTqIScKTHSvL8dt0m&si=bbp9Qc9xVI-1Tctj) 

## Prerequisites

Before you begin, ensure you have met the following requirements:

- **Docker**: You need to have Docker installed on your system. Follow the official [Docker installation guide for Ubuntu](https://docs.docker.com/engine/install/ubuntu/).
- **Visual Studio Code**: Install Visual Studio Code, if you haven't already.
- **Docker Extension for Visual Studio Code**: Install the Docker extension from the Visual Studio Code marketplace.

# Getting Started

Follow these steps to get your project up and running in a Docker environment.

### 1. Clone the Repository
```git clone -b docker https://github.com/Renbago/autonomus_vehicle.git```

### 2. Build Docker Images

1.You can use this command if you suspect that a cached layer is causing issues (e.g., dependencies not updating properly), or if you made changes that Docker’s layer cache might not recognize (such as updates to apt-get or similar commands inside the Dockerfile);

```docker-compose build --no-cache```

2.If you've just made changes to your application code or Dockerfile and want to rebuild the image and start the containers is sufficient and faster;

```docker-compose up -d --build``` 

### 3. Start Docker Containers:
```docker-compose up```
if you want access from terminal run this command ```docker exec -it autonomous_ws /bin/bash```

### 4. Attach Visual Studio Code to Docker Container

1.Open Visual Studio Code.

2.Open the Docker extension panel (usually on the left sidebar).

3.Find the running container for your project.

4.Right-click on the container and select **Attach Visual Studio Code**.

This will open a new instance of Visual Studio Code that is connected to the file system within the Docker container, allowing you to develop and debug your application directly inside the container.

# For running the project:
at one terminal:
```roslaunch sim_pkg map_with_car.launch``` 
the second terminal:
```rosrun example mpc_node```
The mpc version has not been fully finish. and the python side has not been cleaned but for want to see the project instead of ```mpc_node```
you can run ```rosrun example mpc.py```

#

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
