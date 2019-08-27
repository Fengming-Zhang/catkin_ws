cmake_minimum_required(VERSION 2.8.3)
project(we_finger_motor_manager)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  roscpp
  rospy
  std_msgs
  tf
  we_console
  we_log
  we_msgs
)

find_library(LOG4CXX_LIBRARY log4cxx)

generate_dynamic_reconfigure_options(cfg/MotorManagerParameter.cfg)


catkin_package(
  CATKIN_DEPENDS 
    dynamic_reconfigure 
    geometry_msgs 
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

add_executable(we_finger_motor_manager
    src/SerialCom.cpp 
    src/SerialCom.h
    src/SerialComMotor.cpp 
    src/SerialComMotor.h
    src/CanOpen.cpp 
    src/CanOpen.h
    src/CanMotor.cpp 
    src/CanMotor.h
    src/ArmPlan.cpp
    src/MotorManagerLog.cpp 
    src/MotorManagerLog.h
    src/DeviceInterface.cpp 
    src/DeviceInterface.h
    src/MotorManagerNode.cpp 
    src/MotorManagerNode.h
    src/WEStopButton.cpp 
    src/WEStopButton.h
    src/ConfigManager.cpp 
    src/ConfigManager.h
    src/pantiltBroadcaster.cpp 
    src/pantiltBroadcaster.h
    src/main.cpp

    src/RS485.h
    src/RS485.cpp
    src/ProValve.h
    src/ProValve.cpp
    src/PullPressSensor.h
    src/PullPressSensor.cpp
)

add_dependencies(we_finger_motor_manager we_msgs_generate_messages_cpp)

target_link_libraries(we_finger_motor_manager ${catkin_LIBRARIES} usbcan)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS we_can_motor_manager we_can_motor_manager_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_we_can_motor_manager.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
