cmake_minimum_required(VERSION 2.8.3)
project(gecko_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################
## Generate messages in the 'msg' folder
add_message_files(
  FILES
  MbedTx.msg
  BaseVelocity.msg
  FlipperVelocity.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
#add_library(
#  gecko_control
#  src/${PROJECT_NAME}/GeckoControl.h
#)


## gecko_control_nodeという名前の実行可能ファイルを
## src/gecko_control_node.cppから生成
add_executable(gecko_control_node
  src/gecko_control_node.cpp
)

#GeckoControlライブラリとしてGeckoControlクラスの実装部を登録
add_library(GeckoControl
  src/GeckoControl.cpp
)

## ライブラリを実行可能ファイルにリンク
target_link_libraries(gecko_control_node
  ${catkin_LIBRARIES}
  GeckoControl
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(gecko_control_node
  gecko_control_generate_messages_cpp
)
