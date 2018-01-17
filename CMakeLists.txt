cmake_minimum_required(VERSION 2.8.3)
project(gpio_monitor)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

find_library(wiringPi wiringPi ./src/wiringPi)


################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
    FILES
    gpio_input.msg
    gpio_output.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gpio_monitor
  CATKIN_DEPENDS roscpp std_msgs message_runtime
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(gpio_monitor src/gpio_monitor.cpp)
add_dependencies(gpio_monitor ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(gpio_monitor ${catkin_LIBRARIES} ${wiringPi})

#############
## Install ##
#############

#############
## Testing ##
#############

