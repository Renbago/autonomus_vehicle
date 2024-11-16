**You can easily run this project buy using docker branch instead of creating locally.**

# From leader of KOU-Mekatronom
Hi,

Attached are the codes I developed for the KOU-Mekatronom team, which I was a part of from 2021 to 2024, and where I served as the team captain since 2022. Last year, we were accepted into the BOSCH Future Mobility Challenge competition, held in Romania. Our team was the only one from Turkey to reach the finals (top 24) among 160 international teams.

This project includes everything I have developed since May 2023. While working on this project, I was also employed at [Saha Robotic](https://www.linkedin.com/company/saha-robotik/mycompany/), so I could only develop it during the evenings after work and on weekends. Because of this, I am aware that some parts of the project are hard-coded for specific competition scenarios and are not fully complete. However, I believe I did everything I could within the time I had, and I have no regrets.

Currently, the project is written in Python, but I have started porting it to C++ with cleaner code. Soon, I will stop contributing to this project without further updates. The reason is that I have graduated and would like to move on to new challenges, passing this project on to new team members.

This project works in both real-world applications and simulation. In the real system, we used ZED 2i, an encoder, and an IMU for localization, and we plan to add UWB for global reference. Additionally, instead of LiDAR, we used point cloud data from the ZED for obstacle detection, but all other codes remain the same.

**Thanks for reading**

# Project explanation

The autonomous driving  and connectivity algorithms on 1/10 scale vehicles, provided by the company, to navigate in a designated environment simulating a miniature smart city. This project designed for the competation named as BOSCH FUTURE MOBILITY CHALLANGE. 

# How the algorithms are work ?

We have [node graph list](https://github.com/Renbago/autonomus_vehicle/blob/main/src/example/config/fixed2.graphml) (also you can create your node base graph system)
we are selecting the [start and targetnode](https://github.com/Renbago/autonomus_vehicle/blob/ba1dc0e1d733606ee26514bc1f55c89231d02a76/src/example/include/mekatronom/MpcNode.hpp#L94-L95) (It will define from launch file later currently we are changing in python code and header file as i mentioned that.) and [djikstra](https://github.com/Renbago/autonomus_vehicle/blob/main/src/example/include/mekatronom/utilities/djikstra.h) is solving the closest path. You can add [excluded nodes](https://github.com/Renbago/autonomus_vehicle/blob/ba1dc0e1d733606ee26514bc1f55c89231d02a76/src/example/include/mekatronom/MpcNode.hpp#L97C1-L97C55). If there is a [obstacle](https://github.com/Renbago/autonomus_vehicle/tree/main/src/obstacle_detector) we are detecting this center_x, center_y and velocity then giving this input to [mpc_running](https://github.com/Renbago/autonomus_vehicle/blob/ba1dc0e1d733606ee26514bc1f55c89231d02a76/src/example/include/mekatronom/utilities/mpc_running.h#L181) if the obstacles position matches with the node graph system we are detecting it as obstacle and calling djikstra again. If the node has pass_through, we are crossing the left lane but if its set **false** we are waiting the until [obstacle moved](https://github.com/Renbago/autonomus_vehicle/blob/5e3e2af504190099f591417b3469074eb4eb44af/src/example/include/mekatronom/utilities/traffic_sign_manager.h#L41C1-L41C59) if obstacles moved we are triggering djikstra again and the path will get update. Non linear mpc will be follow the next node_id. 

There is a few behaviour states and will be updated coming data's from yolo and obstacles else the state will be ```keep_lane``` we have different parametrs for different states those parameters has been [tuned](https://github.com/Renbago/autonomus_vehicle/blob/ba1dc0e1d733606ee26514bc1f55c89231d02a76/src/example/include/mekatronom/utilities/mpc_start_setting.h#L54-L84) for real time system. And also there is a **behaviour_timer** section. This working same logic with **watchdog_timer** if something went wrong or some behaviours should be executed, by **behaviour_callback** we are able to execute those scenerios. 

# Whats included ? 
I also wanted to share The foundation of this code was inspired by and further developed upon the work in the following GitHub repositories:

This project using [Non-linear MPC with CasADi](https://github.com/MMehrez/MPC-and-MHE-implementation-in-MATLAB-using-Casadi/tree/master/workshop_github), [Dynamic/static obstacle detection](https://github.com/jk-ethz/obstacle_detector), [robot_localization](https://github.com/cra-ros-pkg/robot_localization), Rviz visualization, djikstra and [yolo](https://github.com/ultralytics/ultralytics). 

The all of the implementation's which i mentioned are currently work on [mpc.py](https://github.com/Renbago/autonomus_vehicle/blob/devel/src/example/src/mpc.py),
but the all of the python codes has not been cleaned and a bit dirty code. But while preparing the competation I was not care of the clean writing. Mostly settings are inside of ```mpc.py``` you can just execute it. 

**The cpp part is not fully finished. Currently we are supporting those futures;**
Dynamic Static obstacle detection, Non linear MPC with waypoint base system, robot_localization path regeneration by obstacles. Soon traffic_sign_scenerios will be wroted so selected nodes or the table data's from yolo we will be change the mpc parameters.

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
```docker-compose up```,

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

or

```rosrun example mpc.py```

# IMPORTANT
The ```mpc_node.cpp``` version has not been fully finished, you can run ```rosrun example mpc.py``` when launched ```map_with_car.launch``` also obstacle_detector will be executed.

#

# Outputs from Gazebo

<img src="https://github.com/ECC-BFMC/Simulator/blob/main/Picture1.png" width=30% height=30%>

# From new parkour:

<img src="https://github.com/KOU-Mekatronom/Simulator/blob/main/parkour.png" width=30% height=30%>

# From added RVIZ:

<img src="https://github.com/KOU-Mekatronom/Simulator/blob/main/rviz.png" width=30% height=30%>

# Obstacle detection

<img src="https://github.com/KOU-Mekatronom/Simulator/blob/main/obstacle_detection.png" width=30% height=30%>

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
