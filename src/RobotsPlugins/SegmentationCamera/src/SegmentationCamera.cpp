/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <SegmentationCamera.h>

#include <RobotsPlugins/Utils/Utils.hpp>

#include <gazebo/physics/Link.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/rendering/Distortion.hh>
#include <gazebo/sensors/SensorManager.hh>

GZ_REGISTER_WORLD_PLUGIN(gazebo::SegmentationCamera)

using namespace Eigen;
using namespace RobotsPlugins::Utils;
using namespace gazebo;
using T = Transform<double, 3, Affine>;


SegmentationCamera::~SegmentationCamera()
{
    depth_frame_update_connection_.reset();
    world_update_connection_.reset();

    if(depth_buffer_)
    {
        delete[] depth_buffer_;
    }
}


void SegmentationCamera::Load(physics::WorldPtr world, sdf::ElementPtr sdf)
{
    /* Load parameters. */
    double depth_camera_fx;
    double depth_camera_fy;

    /* FIXME: some of these should be available using the APIs. */
    if (!load_parameter_from_sdf(sdf, "cameraSensorName", camera_sensor_name_))
        return;

    if (!load_parameter_from_sdf(sdf, "cameraSensorFocal", depth_camera_fx))
        return;

    if (!load_parameter_from_sdf(sdf, "cameraModelName", camera_model_name_))
        return;

    if (!load_parameter_from_sdf(sdf, "cameraLinkName", camera_link_name_))
        return;

    if (!load_parameter_from_sdf(sdf, "objectModelName", object_model_name_))
        return;

    if (!load_parameter_from_sdf(sdf, "objectMeshPath", object_mesh_path_))
        return;

    depth_camera_fy = depth_camera_fx;

    /* Get sensor and associated depth camera. */
    depth_camera_sensor_ = std::dynamic_pointer_cast<sensors::DepthCameraSensor>(sensors::SensorManager::Instance()->GetSensor(camera_sensor_name_));
    if (depth_camera_sensor_ == nullptr)
    {
        std::cerr << log_name_ << "::Load. Error: cannot find depth camera sensor " << camera_sensor_name_ << std::endl;
        return;
    }

    depth_camera_ = depth_camera_sensor_->DepthCamera();
    if (depth_camera_ == nullptr)
    {
        std::cerr << log_name_ << "::Load. Error: cannot get depth camera from sensor " << camera_sensor_name_ << std::endl;
        return;
    }

    /* Get model containing the camera. */
    camera_model_ = world->ModelByName(camera_model_name_);
    if (camera_model_ == nullptr)
    {
        std::cerr << log_name_ << "::Load. Error: cannot find model " << camera_model_name_ << " containing the depth camera" << std::endl;
        return;
    }

    /* Store depth image size. */
    width_ = depth_camera_sensor_->ImageWidth();
    height_ = depth_camera_sensor_->ImageHeight();

    /* Get model containing the object. */
    object_model_ = world->ModelByName(object_model_name_);
    if (object_model_ == nullptr)
    {
        std::cerr << log_name_ << "::Load. Error: cannot find model " << object_model_name_ << " containing the object" << std::endl;
        return;
    }

    /* Initialize the segmentation camera. */
    RobotsIO::Camera::CameraParameters parameters;
    parameters.width(width_);
    parameters.height(height_);
    parameters.fx(depth_camera_fx);
    parameters.fy(depth_camera_fy);
    parameters.cx(width_ / 2.0);
    parameters.cy(height_ / 2.0);

    segmentation_ = std::unique_ptr<RobotsIO::Camera::SegmentationCamera>
    (
        new RobotsIO::Camera::SegmentationCamera(parameters, object_mesh_path_, 0.02)
    );

    /* Initialize output. */
    /* FIXME: should not work with more than one object of the same model loaded in Gazebo. */
    output_ = std::unique_ptr<RobotsIO::Utils::YarpImageOfProbe<yarp::sig::PixelMono>>
    (
        new RobotsIO::Utils::YarpImageOfProbe<yarp::sig::PixelMono>("/" + object_model_name_ + "/segmentation:o")
    );

    /* Bind to new depth frame event. */
    auto bind = boost::bind(&SegmentationCamera::on_depth_frame, this, _1, _2, _3, _4, _5);
    depth_frame_update_connection_ = depth_camera_->ConnectNewDepthFrame(bind);

    /* Bind to world update event. */
    world_update_connection_ = event::Events::ConnectWorldUpdateBegin(std::bind(&SegmentationCamera::on_world_frame, this));
}


Eigen::Transform<double, 3, Eigen::Affine> SegmentationCamera::convert(const ignition::math::Pose3d& pose)
{
    ignition::math::Vector3d position = pose.Pos();
    ignition::math::Quaterniond rotation = pose.Rot();

    T transform;
    transform = Translation<double, 3>(position.X(), position.Y(), position.Z());
    transform.rotate(Quaterniond(rotation.W(), rotation.X(), rotation.Y(), rotation.Z()));

    return transform;
}


Eigen::Transform<double, 3, Eigen::Affine> SegmentationCamera::get_object_transform()
{
#if GAZEBO_MAJOR_VERSION >= 8
    ignition::math::Pose3d pose = object_model_->GetLink()->WorldPose();
#else
    gazebo::math::Pose gazebo_pose = object_model__->GetLink()->GetWorldPose();
    ignition::math::Pose3d pose = gazebo_pose.Ign();
#endif

    return convert(pose);
}


Eigen::Transform<double, 3, Eigen::Affine> SegmentationCamera::get_camera_transform()
{
#if GAZEBO_MAJOR_VERSION >= 8
    ignition::math::Pose3d pose = camera_model_->GetLink(camera_link_name_)->WorldPose();
#else
    gazebo::math::Pose gazebo_pose = camera_model_->GetLink(camera_link_name_)->GetWorldPose();
    ignition::math::Pose3d pose = gazebo_pose.Ign();
#endif

    return convert(pose);
}


void SegmentationCamera::on_world_frame()
{
    std::lock_guard<std::mutex> lock(depth_mutex_);

    if ((depth_buffer_ == nullptr) || !new_depth_)
        return;

    const MatrixXf depth = Map<const Matrix<float, -1, -1, RowMajor>>(depth_buffer_, height_, width_);

    cv::Mat mask;
    bool valid_mask = false;
    std::tie(valid_mask, mask) = segmentation_->mask(depth, get_camera_transform().inverse() * get_object_transform());

    if (valid_mask)
        output_->set_data(mask);

    new_depth_ = false;
}


void SegmentationCamera::on_depth_frame(const float* image, unsigned int width, unsigned int height, unsigned int depth, const std::string& format)
{
    if(depth_camera_sensor_->IsActive())
    {
        std::lock_guard<std::mutex> lock(depth_mutex_);

        if (depth_buffer_ == nullptr)
            depth_buffer_ = new float[width_ * height_];

        memcpy(depth_buffer_, image, width_ * height_ * sizeof(float));

        new_depth_ = true;
    }
}
