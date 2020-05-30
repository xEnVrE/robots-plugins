#===============================================================================
#
# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

set(PLUGIN_TARGET_NAME robots-plugins-segmentation-camera)

find_package(GazeboYARPPlugins REQUIRED)

find_package(Gazebo REQUIRED)

find_package(RobotsIO REQUIRED)

set(${PLUGIN_TARGET_NAME}_HDR
    include/SegmentationCamera.h
)

set(${PLUGIN_TARGET_NAME}_SRC
    src/SegmentationCamera.cpp
)

add_library(${PLUGIN_TARGET_NAME} SHARED ${${PLUGIN_TARGET_NAME}_HDR} ${${PLUGIN_TARGET_NAME}_SRC})

target_include_directories(${PLUGIN_TARGET_NAME}
                           PRIVATE
                           ${CMAKE_CURRENT_SOURCE_DIR}/include
                           ${GAZEBO_INCLUDE_DIRS}
)

target_link_libraries(${PLUGIN_TARGET_NAME}
                      PRIVATE
                      robots_plugins_utils
                      GazeboYARPPlugins::gazebo_yarp_singleton
                      RobotsIO::RobotsIO
                      ${GAZEBO_LIBRARIES}
)

yarp_install(TARGETS ${PLUGIN_TARGET_NAME}
             COMPONENT Runtime
             LIBRARY DESTINATION  ${CMAKE_INSTALL_LIBDIR}
             ARCHIVE DESTINATION  ${CMAKE_INSTALL_LIBDIR})
