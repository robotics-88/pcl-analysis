cmake_minimum_required(VERSION 3.8)
project(pcl_analysis)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

## Find dependencies
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(sensor_msgs REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

include_directories(include 
  ${OpenCV_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
)

link_directories(${OpenCV_LIB_DIR} ${PCL_LIB_DIR})

###########
## Build ##
###########

#Color projection pkgs
#add_executable(color_pc_sync_modified_node src/color_pc_sync_node_modified.cpp)
#target_link_libraries(color_pc_sync_modified_node ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} )

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/pcl_analysis.cpp
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node src/pcl_analysis_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
ament_target_dependencies(${PROJECT_NAME}_node 
  pcl_ros
  sensor_msgs
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
)

#############
## Install ##
#############

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_pcl-analysis.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
