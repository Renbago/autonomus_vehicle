cmake_minimum_required(VERSION 3.0.2)
project(example)

# Ensure C++17 is used
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_WARN_DEPRECATED OFF)
set(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION OFF)

# Standard C++ settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-std=c++17 -Wall -Wextra)
endif()

# Add subdirectory for Casadi
add_subdirectory(lib/casadi)

# Find required packages
find_package(Casadi REQUIRED)
find_package(pugixml REQUIRED)
find_package(Eigen3 REQUIRED)

message(STATUS "Casadi include dir: ${Casadi_INCLUDE_DIRS}")
message(STATUS "Casadi libraries: ${Casadi_LIBRARIES}")

# Find catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  rospy
  roscpp
  sensor_msgs
  utils
  cv_bridge
  image_transport
  message_generation
)

# Add message files
add_message_files(
  FILES
  MekatronomYolo.msg
)

# Generate messages
generate_messages(
  DEPENDENCIES
  std_msgs
)

# Catkin specific configuration
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS geometry_msgs std_msgs rospy utils roscpp sensor_msgs message_runtime
  DEPENDS Eigen3
)

# Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# Add executable
add_executable(mpc_node scripts/mpc_node.cpp scripts/MpcNode.cpp)
add_dependencies(mpc_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Link libraries
target_link_libraries(mpc_node 
  ${catkin_LIBRARIES}
  casadi
  ${Casadi_LIBRARIES}
  ${OpenCV_LIBRARIES}
  pugixml
)

# Installation
install(TARGETS mpc_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Optionally install config and launch directories
# install(DIRECTORY config launch
#   DESTINATION share/${PROJECT_NAME}
# )
