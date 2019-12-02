# Jetson CSI camera node(let)

The node(let) is based on the [JetsonHacks' Jetson Nano CSI repository](https://github.com/JetsonHacksNano/CSI-Camera). As such, it only supports Raspberry Pi v2 cameras (and others that are supported by Jetson Nano). Additionally, it requires OpenCV with GStreamer support (see [JetsonHacks' OpenCV build scripts](https://github.com/JetsonHacksNano/buildOpenCV) to build one).

## Parameters

* `cap_width`, `cap_height` (*int*, default: 1280, 720) - Width and height of the captured image. **These parameters only affect the capturing mode!** Be sure to set them to one of the supported modes for your camera (Raspberry Pi camera v2 supports 3280x2464@21fps, 3280x1848@28fps, 1920x1080@30fps, 1280x720@60fps).
* `width`, `height` (*int*, default: 640, 480) - Width and height of the output image.
* `fps` (*int*, default: 60) - Desired framerate/publish rate [Hz].
* `flip_method` (*int*, default 0) - Image rotation/flipping. `0` for no rotation, see detailed description [at JetsonHacks](https://github.com/JetsonHacksNano/CSI-Camera#gstreamer-parameter).
* `frame_id` (*string*, default: "main_camera_optical")- Camera frame name.
* `capture_delay` (*double*, default: 0) - Additional time (in seconds) that is **substracted** from the timestamp. Allows to compensate for capture delays.
* `topic_name` (*string*, default: "image_raw") - Image topic name.
* `camera_info_url` (*string*, default: "") - URL pointing to the camera info yaml file.

## Nodelet

The node is meant to run as a nodelet (`jetson_camera/JetsonCameraNodelet`). In fact, the node is basically a standalone simplified nodelet manager.
