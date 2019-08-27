cmake_minimum_required(VERSION 2.8.3)
project(gpsr_gen)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
  we_console
  we_log
  we_msgs
)

find_library(LOG4CXX_LIBRARY log4cxx)

catkin_package(
  CATKIN_DEPENDS 
    dynamic_reconfigure 
    geometry_msgs 
    nav_msgs 
    roscpp 
    rospy 
    std_msgs 
    tf 
    we_console 
    we_log 
    we_msgs
)
###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})

add_executable(gpsr_gen 
    src/bigobjects.cpp
    src/pose.cpp
    src/furnitures.cpp
    src/rooms.cpp
    src/gpsrgen.cpp
    src/main.cpp
    src/bigobjects.h
    src/pose.h
    src/furnitures.h
    src/rooms.h
    src/gpsrgen.h
    src/entity.h
)

add_dependencies(gpsr_gen we_msgs_generate_messages_cpp)

target_link_libraries(gpsr_gen ${catkin_LIBRARIES})

