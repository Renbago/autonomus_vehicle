# Use the ROS Noetic base image
FROM osrf/ros:noetic-desktop-full

# Set the default shell to bash
SHELL ["bash", "-c"]

# Define the USERNAME variable
ARG USERNAME=baha

# Create the user with the USERNAME variable
RUN useradd -m -s /bin/bash $USERNAME

# Allow the user to use sudo without a password
RUN echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Set ownership of the home directory to the new user
RUN mkdir -p /home/$USERNAME && chown -R $USERNAME:$USERNAME /home/$USERNAME

# Install necessary tools and dependencies
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


USER $USERNAME

WORKDIR /home/$USERNAME

RUN git clone --recurse-submodules https://github.com/casadi/casadi.git

WORKDIR /home/$USERNAME/casadi
RUN mkdir -p build && cd build && cmake -DWITH_IPOPT=true .. && sudo make install && sudo apt update

# Set up bash-completion
RUN echo "if [ -f /etc/bash_completion ]; then" >> ~/.bashrc
RUN echo "  . /etc/bash_completion" >> ~/.bashrc
RUN echo "fi" >> ~/.bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> /home/baha/.bashrc
WORKDIR /home/$USERNAME

# Set up Catkin workspace
RUN mkdir -p autonomus_ws/src && chown -R $USERNAME:$USERNAME autonomus_ws

# Clone the required repositories if needed
RUN cd autonomus_ws/src && git clone -b devel https://github.com/Renbago/autonomus_vehicle.git
RUN cd /home/$USERNAME/autonomus_ws && \
    source /opt/ros/noetic/setup.bash && \
    catkin_make

RUN source /opt/ros/noetic/setup.bash 

# Set environment variables for ROS and Gazebo
RUN echo 'export ROS_PACKAGE_PATH="/home/'$USERNAME'/autonomus_ws/src/autonomus_vehicle/src:$ROS_PACKAGE_PATH"' >> ~/.bashrc
RUN echo 'export GAZEBO_MODEL_PATH="/home/'$USERNAME'/autonomus_ws/src/autonomus_vehicle/src/models_pkg:$GAZEBO_MODEL_PATH"' >> ~/.bashrc
RUN echo 'source /home/'$USERNAME'/autonomus_ws/devel/setup.bash' >> ~/.bashrc