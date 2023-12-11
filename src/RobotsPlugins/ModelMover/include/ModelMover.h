/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSPLUGINS_MODELMOVER_H
#define ROBOTSPLUGINS_MODELMOVER_H

#include <RobotsIO/Utils/TransformYarpPort.h>
#include <RobotsIO/Utils/YarpVectorOfProbe.hpp>

#include <gazebo/common/Plugin.hh>

#include <Eigen/Dense>

#include <string>

namespace gazebo
{
    class ModelMover;
}

class gazebo::ModelMover : public ModelPlugin
{
public:
    ~ModelMover();

    /**
     * Store pointer to the visual.
     */
    void Load(physics::ModelPtr model, sdf::ElementPtr);

private:
    /**
     * World update callback.
     */
    void on_world_frame();

    /**
     * YARP Transform client.
     * FIXME: Extend support to ROS.
     */
    std::unique_ptr<RobotsIO::Utils::TransformYarpPort> transform_in_;

    std::unique_ptr<RobotsIO::Utils::YarpVectorOfProbe<double, Eigen::Transform<double, 3, Eigen::Affine>>> transform_out_;
    Eigen::Transform<double, 3, Eigen::Affine> transform_out_storage_;

    /**
     * Pointer to the model
     */
    gazebo::physics::ModelPtr model_;

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

    const std::string log_name_ = "ModelMover";
};

#endif

/* ROBOTSPLUGINS_MODELMOVER_H */
