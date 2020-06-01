/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <ModelMover.h>

#include <Eigen/Dense>

#include <GazeboYarpPlugins/common.h>

#include <RobotsPlugins/Utils/Utils.hpp>

#include <boost/bind.hpp>

#include <gazebo/physics/Model.hh>
#include <gazebo/common/Events.hh>

#include <ignition/math/Pose3.hh>

GZ_REGISTER_MODEL_PLUGIN(gazebo::ModelMover)

using namespace Eigen;
using namespace RobotsIO::Utils;
using namespace gazebo;


ModelMover::~ModelMover()
{}


void ModelMover::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
    /* Store pointers. */
    model_ = model;
    sdf_ = sdf;

    /* Store model name. */
    model_name_ = model_->GetName();

    /* Initialize input port. */
    transform_ = std::unique_ptr<TransformYarpPort>
    (
        new TransformYarpPort("/" + model_name_ + "/model-mover/pose:i")
    );

    /* Bind update callback . */
    auto bind = boost::bind(&ModelMover::on_world_frame, this);
    renderer_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(bind);
}


void ModelMover::on_world_frame()
{
    /* Get the pose from the input port. */
    if (!transform_->freeze())
        return;

    Eigen::Transform<double, 3, Affine> transform = transform_->transform();


    /* Set the pose .*/
    Quaterniond quaternion(transform.rotation());
    ignition::math::Pose3d pose(transform.translation()(0), transform.translation()(1), transform.translation()(2),
                                quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z());

#if GAZEBO_MAJOR_VERSION >= 8
    ignition::math::Pose3d& gazebo_pose = pose;
#else
    gazebo::math::Pose gazebo_pose(pose);
#endif
    model_->SetWorldPose(gazebo_pose);
}
