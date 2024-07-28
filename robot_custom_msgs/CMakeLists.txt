cmake_minimum_required(VERSION 3.0.2)
project(robot_custom_msgs)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)


add_message_files(
  FILES
  gui_msg.msg
)  


generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
 CATKIN_DEPENDS message_runtime roscpp rospy std_msgs
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

