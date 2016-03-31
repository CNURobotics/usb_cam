usb_cam [![Build Status](https://api.travis-ci.org/bosch-ros-pkg/usb_cam.png)](https://travis-ci.org/bosch-ros-pkg/usb_cam)
=======

#### A ROS Driver for V4L USB Cameras
This package is based off of V4L devices specifically instead of just UVC.

For full documentation, see [the ROS wiki](http://ros.org/wiki/usb_cam).

This version includes the ability to send a soft synchronization topic from one camera to others.  The cameras will attempt to grab their next image based on a desired framerate given the time stamp of the master camera.

[Doxygen](http://docs.ros.org/indigo/api/usb_cam/html/) files can be found on the ROS wiki.

### License
usb_cam is released with a BSD license. For full terms and conditions, see the [LICENSE](LICENSE) file.

### Authors
See the [AUTHORS](AUTHORS.md) file for a full list of contributors.
