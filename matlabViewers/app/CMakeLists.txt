# Copyright: 2013 iCub Facility - Istituto Italiano di Tecnologia
# Author: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.m)

find_package(YARP)
find_package(ICUBcontrib)
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH} ${ICUBCONTRIB_MODULE_PATH})
include(YarpInstallationHelpers)
include(ICUBcontribHelpers)
icubcontrib_set_default_prefix()

yarp_install(FILES ${scripts} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/matlabViewers)
                           
