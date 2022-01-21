cmake_minimum_required(VERSION 3.5)
project(glim_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")
set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Choose the type of build." FORCE)
set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(glim REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Iridescence REQUIRED)

ament_auto_add_library(glim_ros SHARED
  src/glim_ros/raw_points.cpp
  src/glim_ros/glim_ros.cpp
)
target_include_directories(glim_ros PUBLIC
  include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${GTSAM_INCLUDE_DIRS}
  ${Iridescence_INCLUDE_DIRS}
)
target_link_libraries(glim_ros
  ${OpenCV_LIBRARIES}
  ${GTSAM_LIBRARIES}
  ${Iridescence_LIBRARIES}
)
rclcpp_components_register_nodes(glim_ros "glim_ros")


ament_auto_add_executable(glim_rosnode
  src/glim_rosnode.cpp
)
target_link_libraries(glim_rosnode
  glim_ros
)

ament_auto_add_executable(glim_rosbag
  src/glim_rosbag.cpp
)
target_link_libraries(glim_rosbag
  glim_ros
)

ament_auto_add_executable(offline_viewer
  src/offline_viewer.cpp
)
target_include_directories(offline_viewer PUBLIC
  include
  ${EIGEN3_INCLUDE_DIR}
  ${GTSAM_INCLUDE_DIRS}
  ${Iridescence_INCLUDE_DIRS}
)
target_link_libraries(offline_viewer
  ${GTSAM_LIBRARIES}
  ${Iridescence_LIBRARIES}
)

install(DIRECTORY config DESTINATION share/glim_ros)

ament_auto_package()
