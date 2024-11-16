FROM osrf/ros:noetic-desktop-full

SHELL ["bash", "-c"]

ARG USERNAME=Renbago
RUN useradd -m -s /bin/bash $USERNAME && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers && \
    chown -R $USERNAME:$USERNAME /home/$USERNAME

USER root

RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    coinor-libipopt-dev \
    gfortran \
    python3-rosdep \
    apt-transport-https \
    ca-certificates \
    sudo \
    git \
    python3-pip \  
    nlohmann-json3-dev \
    ros-noetic-geometry-msgs \
    ros-noetic-std-msgs \
    ros-noetic-rospy \
    ros-noetic-roscpp \
    ros-noetic-sensor-msgs \
    ros-noetic-message-generation \
    ros-noetic-message-runtime \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    libeigen3-dev \
    ros-noetic-tf2-sensor-msgs \
    libpugixml-dev \
    bash-completion \
    ros-noetic-geographic-msgs \
    libgeographic-dev \
    && rm -rf /var/lib/apt/lists/*

# CasaDI installation
WORKDIR /home/$USERNAME
USER $USERNAME
RUN git clone --recurse-submodules https://github.com/casadi/casadi.git
WORKDIR /home/$USERNAME/casadi
RUN mkdir -p build && cd build && cmake -DWITH_IPOPT=true .. && make -j$(nproc)
USER root
RUN cd /home/$USERNAME/casadi/build && make install

#This casadi is for python version i did not include the local path for pyton codes.
RUN pip3 install casadi  

# Setting up Catkin workspace
WORKDIR /home/$USERNAME
RUN mkdir -p autonomus_ws/src && chown -R $USERNAME:$USERNAME autonomus_ws
RUN cd autonomus_ws/src && git clone -b devel https://github.com/Renbago/autonomus_vehicle.git
WORKDIR /home/$USERNAME/autonomus_ws
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; catkin_make'

# Here is setting the necessery gazebo and ros package  and also chmod+x all of it.
USER $USERNAME
RUN echo "if [ -f /etc/bash_completion ]; then" >> ~/.bashrc && \
    echo "  . /etc/bash_completion" >> ~/.bashrc && \
    echo "fi" >> ~/.bashrc && \
    echo "source /opt/ros/noetic/setup.bash" >> /home/$USERNAME/.bashrc && \
    echo 'export ROS_PACKAGE_PATH="/home/'$USERNAME'/autonomus_ws/src/autonomus_vehicle/src:$ROS_PACKAGE_PATH"' >> ~/.bashrc && \
    echo 'export GAZEBO_MODEL_PATH="/home/'$USERNAME'/autonomus_ws/src/autonomus_vehicle/src/models_pkg:$GAZEBO_MODEL_PATH"' >> ~/.bashrc && \
    echo 'source /home/'$USERNAME'/autonomus_ws/devel/setup.bash' >> ~/.bashrc && \
    find /home/$USERNAME/autonomus_ws/src/autonomus_vehicle -type f -name "*.py" -exec chmod +x {} \; && \
    find /home/$USERNAME/autonomus_ws/src/autonomus_vehicle -type f -name "*.cpp" -exec chmod +x {} \;
