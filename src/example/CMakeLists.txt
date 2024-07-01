cmake_minimum_required(VERSION 3.0.2)
set(CMAKE_CXX_STANDARD 17)
project(example)
set(CMAKE_WARN_DEPRECATED OFF)
set(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION OFF)
# Standart C++ setlemeleri
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-w)
endif()
add_subdirectory(ext/casadi)
add_compile_options(-std=c++17)
find_package(Casadi REQUIRED)
find_package(pugixml REQUIRED)
find_package(Eigen3 REQUIRED)

message(STATUS "Casadi include dir: ${Casadi_INCLUDE_DIRS}")
message(STATUS "Casadi libraries: ${Casadi_LIBRARIES}")

# Bağımlılıkların bulunması
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

add_message_files(
  FILES
  MekatronomYolo.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs  # Or any other packages containing msgs
)

# Catkin specific configuration
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS geometry_msgs std_msgs rospy utils roscpp sensor_msgs message_runtime
  DEPENDS Eigen3
)

# İnclude dizinlerini belirt
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  # ${Casadi_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(mpc_node scripts/mpc_node.cpp scripts/MpcNode.cpp)
add_dependencies(mpc_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


target_link_libraries(mpc_node 
  ${catkin_LIBRARIES}
  # "/home/mekatronom/casadi/build/lib/libcasadi.so"
  # "/home/mekatronom/boschmekatronom_ws/BoschMekatronom/src/example/ext/casadi/build/lib/libcasadi_nlpsol_ipopt.so"
  casadi
  ${Casadi_LIBRARIES}
  ${OpenCV_LIBRARIES}
  pugixml
)

# Yüklemek için yapılandırma
install(TARGETS mpc_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Konfigürasyon ve başlatma dizinleri gerekirse yorumdan çıkarılabilir
# install(DIRECTORY config launch
#   DESTINATION share/${PROJECT_NAME}
# )