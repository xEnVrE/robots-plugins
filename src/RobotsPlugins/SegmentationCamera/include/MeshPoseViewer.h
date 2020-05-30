/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSPLUGINS_MESHPOSEVIEWER_H
#define ROBOTSPLUGINS_MESHPOSEVIEWER_H

#include <gazebo/common/Plugin.hh>

#include <yarp/os/Network.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/IFrameTransform.h>
#include <yarp/dev/PolyDriver.h>

#include <string>

namespace gazebo
{
    class MeshPoseViewer;
}

class gazebo::MeshPoseViewer : public VisualPlugin
{
public:
    ~MeshPoseViewer();

    /**
     * Store pointer to the visual.
     */
    void Load(gazebo::rendering::VisualPtr, sdf::ElementPtr);

private:
   /**
    * Extract the name of the model from the name of the visual.
    */
    std::string get_model_name();

    /**
     * Rendering callback.
     */
    void on_render_update();

    /**
     * Instance of yarp::os::Network.
     */
    yarp::os::Network yarp_;

    /**
     * yarp::dev::IFrameTransform driver
     */
    yarp::dev::PolyDriver drv_transform_client_;

    /**
     * yarp::dev::IFrameTransform view of the driver;
     */
    yarp::dev::IFrameTransform* transform_client_;

    /**
     * Pointer to the visual element
     */
    gazebo::rendering::VisualPtr visual_;

    /**
    * Pointer to the sdf associated to the visual
    */
    sdf::ElementPtr sdf_;

    /**
    * Connection to the Render event of Gazebo
    */
    gazebo::event::ConnectionPtr renderer_connection_;

    /**
     * Name of the model where the visual element is contained
     */
    std::string model_name_;

    /**
    * Name of the source frame required to retrieve the current estimate
    */
    std::string source_frame_;

    /**
     * Target frame suffix required to retrieve the current estimate
     */
    std::string target_frame_suffix_;

    const std::string log_name_ = "MeshPoseViewer";
};

#endif

/* ROBOTSPLUGINS_MESHPOSEVIEWER_H */
