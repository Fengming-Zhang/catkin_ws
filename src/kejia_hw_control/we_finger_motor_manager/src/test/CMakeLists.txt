cmake_minimum_required(VERSION 2.8.3)
project(we_can_motor_test)

find_package(Boost REQUIRED)

include_directories(${Boost_INCLUDE_DIRS})
link_directories( ${Boost_LIBRARY_DIRS})

add_executable(we_can_motor_test
    arm.cpp
)


target_link_libraries(we_can_motor_test ${Boost_LIBRARIES} boost_thread usbcan pthread boost_system)


