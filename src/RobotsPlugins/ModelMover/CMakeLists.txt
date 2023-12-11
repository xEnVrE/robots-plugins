#===============================================================================
#
# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

set(PLUGIN_TARGET_NAME robots-plugins-model-mover)

# find_package(YARP REQUIRED
#              COMPONENTS dev os math
# )

find_package(Eigen3 REQUIRED)

find_package(GazeboYARPPlugins REQUIRED)

find_package(Gazebo REQUIRED)

find_package(RobotsIO REQUIRED)

set(${PLUGIN_TARGET_NAME}_HDR
    include/ModelMover.h
)

set(${PLUGIN_TARGET_NAME}_SRC
    src/ModelMover.cpp
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
                      # YARP::YARP_dev
                      # YARP::YARP_math
                      # YARP::YARP_os
                      Eigen3::Eigen
                      ${GAZEBO_LIBRARIES}
)

install(TARGETS ${PLUGIN_TARGET_NAME}
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib
 )
