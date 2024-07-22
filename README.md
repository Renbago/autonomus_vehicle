The autonomous driving  and connectivity algorithms on 1/10 scale vehicles, provided by the company, to navigate in a designated environment simulating a miniature smart city. This project designed for the competation named as BOSCH FUTURE MOBILITY CHALLANGE. Its include Non-linear MPC with CasADi, Dynamic/static obstacle detection, robot_localization, Rviz visualization, djikstra and yolo.  

Installation :


```
git clone https://github.com/Renbago/autonomus_vehicle.git

current_dir=$(pwd)

echo 'export ROS_PACKAGE_PATH="'$current_dir'/src:$ROS_PACKAGE_PATH"' >> ~/.bashrc
echo 'export GAZEBO_MODEL_PATH="'$current_dir'/src/models_pkg:$GAZEBO_MODEL_PATH"' >> ~/.bashrc

source ~/.bashrc

catkin_make
```


<img src="https://github.com/ECC-BFMC/Simulator/blob/main/Picture1.png" width=30% height=30%>

From new parkour:

<img src="https://github.com/KOU-Mekatronom/Simulator/blob/main/parkour.png" width=30% height=30%>

From added RVIZ:

<img src="https://github.com/KOU-Mekatronom/Simulator/blob/main/rviz.png" width=30% height=30%>

Traffic lights plugin:

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


The algorithms also working in real life: (soon will be updated)

 [![Video Title](https://youtube.com/playlist?list=PLDE_vDxu0Gkk-s3ndTqIScKTHSvL8dt0m&si=bbp9Qc9xVI-1Tctj)](https://youtube.com/playlist?list=PLDE_vDxu0Gkk-s3ndTqIScKTHSvL8dt0m&si=bbp9Qc9xVI-1Tctj)


Needs to fix those errors:
```
Couldn't fix it yet. After few catkin_make the workspace will work.

In file included from /home/mekatronom/autonomus_vehicle/src/plugins_pkgs/bno055_plugin/src/bno055_plugin.cpp:1:
/home/mekatronom/autonomus_vehicle/src/plugins_pkgs/bno055_plugin/include/bno055_plugin.hpp:8:10: fatal error: utils/IMU.h: No such file or directory
    8 | #include "utils/IMU.h"
      |          ^~~~~~~~~~~~~
compilation terminated.
make[2]: *** [plugins_pkgs/bno055_plugin/CMakeFiles/bno055_plugin.dir/build.make:63: plugins_pkgs/bno055_plugin/CMakeFiles/bno055_plugin.dir/src/bno055_plugin.cpp.o] Error 1
make[1]: *** [CMakeFiles/Makefile2:6686: plugins_pkgs/bno055_plugin/CMakeFiles/bno055_plugin.dir/all] Error 2
make[1]: *** Waiting for unfinished jobs....

In file included from /home/mekatronom/autonomus_vehicle/src/plugins_pkgs/gps_plugin/src/gps_plugin.cpp:1:
/home/mekatronom/autonomus_vehicle/src/plugins_pkgs/gps_plugin/include/gps_plugin.hpp:10:10: fatal error: utils/localisation.h: No such file or directory
   10 | #include "utils/localisation.h"
      |          ^~~~~~~~~~~~~~~~~~~~~~
compilation terminated.
make[2]: *** [plugins_pkgs/gps_plugin/CMakeFiles/gps_plugin.dir/build.make:63: plugins_pkgs/gps_plugin/CMakeFiles/gps_plugin.dir/src/gps_plugin.cpp.o] Error 1
make[1]: *** [CMakeFiles/Makefile2:7099: plugins_pkgs/gps_plugin/CMakeFiles/gps_plugin.dir/all] Error 2


```