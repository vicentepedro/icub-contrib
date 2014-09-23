# Copyright: (C) 2014 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8)
set(PROJECTNAME funnyPostures)
project(${PROJECTNAME})

find_package(YARP)
find_package(ICUB)
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUB_MODULE_PATH})
include(YarpInstallationHelpers)

find_package(ICUBcontrib)
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})
include(ICUBcontribHelpers)
include(ICUBcontribOptions)
icubcontrib_set_default_prefix()

source_group("Source Files" FILES src/main.cpp)
include_directories(${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS})

add_executable(${PROJECTNAME} src/main.cpp)
target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} ctrlLib icubmod)
install(TARGETS ${PROJECTNAME} DESTINATION bin)
icubcontrib_add_uninstall_target()

add_subdirectory(app)
