cmake_minimum_required(VERSION 3.0.2)
project(tta_dataset_player)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    velodyne_pointcloud
)

catkin_package(
  INCLUDE_DIRS 
  CATKIN_DEPENDS 
    roscpp
    velodyne_pointcloud
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)