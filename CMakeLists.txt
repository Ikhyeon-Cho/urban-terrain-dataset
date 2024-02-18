cmake_minimum_required(VERSION 3.0.2)
project(ku_dataset_ros)

find_package(catkin REQUIRED COMPONENTS
    roscpp
)

catkin_package(
  INCLUDE_DIRS 
  CATKIN_DEPENDS 
    roscpp
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)