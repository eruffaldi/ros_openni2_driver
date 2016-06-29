OpenNI2-rosDriver
=================

OpenNI2-rosDriver is a bridge to ROS implemented as an OpenNI2 driver.

NODE semantics of this Driver: the driver is a shared library hosted inside a non ROS node (e.g. if used in NiViewer) or in a ROS node (e.g. if OpenNI2 is used as a library inside a ROS node for nite/camera. In this case if we plan to use it we need the environment variable ROS2OPENNI2 OR )

Phase 2:
- CameraInfo
- input registered depth from ros

Inspiration to: http://wiki.ros.org/image_pipeline

https://github.com/ros-perception/image_pipeline/blob/indigo/depth_image_proc/src/nodelets/point_cloud_xyzrgb.cpp

ROS Topic Subscribed:
Given the uri: ros://XYZ/ then

XYZ/rgb
XYZ/depth_registered
XYZ/info

ROS Parameters:
queue_size
exact_sync

TODO:
- clarify how node is instantiated (vs original nodelet)
- conversion of the stream images
- clarify which modes are supported aka camera info


ROS Topic Aliases
