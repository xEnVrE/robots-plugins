/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSPLUGINS_SEGMENTATIONCAMERA_H
#define ROBOTSPLUGINS_SEGMENTATIONCAMERA_H

#include <Eigen/Dense>

#include <RobotsIO/Camera/SegmentationCamera.h>
#include <RobotsIO/Utils/YarpImageOfProbe.hpp>

#include <mutex>

#include <gazebo/common/Plugin.hh>
#include <gazebo/rendering/DepthCamera.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/sensors/DepthCameraSensor.hh>


namespace gazebo
{
    class SegmentationCamera;
}

class gazebo::SegmentationCamera : public WorldPlugin
{
public:
    ~SegmentationCamera();

    void Load(physics::WorldPtr world, sdf::ElementPtr) override;

private:
    /**
     * Convert from ignition::math::Pose3d to Eigen::Transform<double, 3, Affine>.
     */
    Eigen::Transform<double, 3, Eigen::Affine> convert(const ignition::math::Pose3d& pose);

    /**
     * Retrieve the current pose of the object in world frame.
     */
    Eigen::Transform<double, 3, Eigen::Affine> get_object_transform();

    /**
     * Retrieve the current pose of the camera in world frame.
     */
    Eigen::Transform<double, 3, Eigen::Affine> get_camera_transform();

    /**
     * World update callback.
     */
    void on_world_frame();

    /**
     * Depth frame update callback.
     */
    void on_depth_frame(const float* image, unsigned int width, unsigned int height, unsigned int depth, const std::string& format);

    /**
     * Depth storage, mutex, signalling and parameters.
     */
    float* depth_buffer_;

    bool new_depth_ = false;

    std::mutex depth_mutex_;

    std::size_t width_;

    std::size_t height_;

    /**
     * Segmentation renderer.
     */
    std::unique_ptr<RobotsIO::Camera::SegmentationCamera> segmentation_;

    /**
     * Pointers to depth camera sensor.
     */
    gazebo::sensors::DepthCameraSensorPtr depth_camera_sensor_;

    gazebo::rendering::DepthCameraPtr depth_camera_;

    /**
     * Pointer to the object model.
     */
    gazebo::physics::ModelPtr object_model_;

    /**
     * Pointer to the camera model.
     */
    gazebo::physics::ModelPtr camera_model_;

    /**
     * Pointers to Gazebo update events.
     */
    gazebo::event::ConnectionPtr depth_frame_update_connection_;

    gazebo::event::ConnectionPtr world_update_connection_;

    std::unique_ptr<RobotsIO::Utils::YarpImageOfProbe<yarp::sig::PixelMono>> output_;

    /**
     * Model and link names/paths.
     */
    std::string object_model_name_;

    std::string object_mesh_path_;

    std::string camera_model_name_;

    std::string camera_link_name_;

    std::string camera_sensor_name_;

    const std::string log_name_ = "SegmentationCamera";
};

#endif

/* ROBOTSPLUGINS_SEGMENTATIONCAMERA_H */
