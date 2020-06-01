# [WIP] robots-plugins

A collection of plugins for Gazebo to experiment with perception and manipulation tasks.

### Dependencies

- [`RobotsIO`](https://github.com/xEnVrE/robots-io)
- [`SuperimposeMesh`](https://github.com/xEnVrE/superimpose-mesh-lib/commits/impl/fastrendering)
  > The specific fork github.com/xenvre/superimpose-mesh-lib/tree/impl/fastrendering is required at the moment
- [`YARP`](https://github.com/robotology/yarp)

### Installation

```
git clone https://github.com/xenvre/robots-plugins.git
cd robots-io
mkdir build
cd build
cmake -DCMAKE_PREFIX_PATH=<installation_path> ../
make install
```

then make sure that `GAZEBO_PLUGIN_PATH` contains `<installation_path>/lib`.

## Available plugins (more coming soon)

### SegmentationCamera (World plugin)

[![Link to video](https://github.com/xEnVrE/robots-plugins/blob/master/assets/gazebo_segmentation_thumb.jpg)](https://youtu.be/7gDSyAUS8Nw)


Given a depth camera within the world and the model of an object of interest, it produces the associated segmentation mask. At the moment, the output is available via a`YARP` port whose content is of type `yarp::sig::ImageOf<PixelMono>`.

Example of usage:

```
<plugin name="segmentation_camera" filename="librobots-plugins-segmentation-camera.so">
    <cameraSensorName>name of the depth sensor in the world </cameraSensorName>
    <cameraSensorFocal>focal length of the camera in pixels</cameraSensorFocal>
    <cameraModelName>name of the model containing the camera</cameraModelName>
    <cameraLinkName>name of the link associated to the camera</cameraLinkName>
    <objectModelName>name of the model containing the object of interest</objectModelName>
    <objectMeshPath>path to the mesh used for the visual of the object</objectMeshPath>
 </plugin>
```
> (the number of parameters will be reduced in following releases, since most of them should be available using `SDF` APIs)


- it is assumed that the pose of the object is that of the canonical link of the model `objectModelName`
- the output port name is hardcoded to `/<objectModelName>/segmentation:o`
- only one instance of a given model per time is supported at  the moment

### ModelMover (Model plugin)

It allows moving a model within the world. At the moment, the input is available via a `YARP` port whose content has to be of type `yarp::sig::Vector`. The vector should contain a `x y z axis_x axis_y axis_z angle` parametrization of the pose of the model.

Example of usage:
```
<plugin name="model_mover filename="librobots-plugins-model-mover.so"></plugin>
```

- the pose is associated to the **canonical** link of the model
- the input port is hardcoded to `/<objectModelName/model-mover/pose:i`

In order to change the pose of the object over time without the effect of the gravity, it is required to disable the dynamic engine for that model using

```
<model name="model_name">
(...)

    <static>true</static>
    
(...)
</model>
```

It is also possible to add this option outside the `SDF` model at the moment of its inclusion in the world, i.e.

```
<world>
(...) 

    <include>
        <uri>model://path/to/model</uri>              
        <static>true</static>
    </include>
        
(...)
</world>
```
