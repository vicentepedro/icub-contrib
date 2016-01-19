# Copyright: (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8)
project(tracker)

find_package(YARP REQUIRED)
list(APPEND CMAKE_MODULE_PATH YARP_MODULE_PATH)
find_package(OpenCV 3.0.0 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS}) 
add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES} ${YARP_LIBRARIES})

