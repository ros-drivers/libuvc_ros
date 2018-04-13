^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package libuvc_camera
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.10 (2018-04-13)
-------------------
* Merge pull request `#46 <https://github.com/ros-drivers/libuvc_ros/issues/46>`_ from k-okada/master
  set timestamp
* set only when frame->capture_time is 0
* set ros::Time::now() because libuvc does not set capture_time (https://github.com/ktossell/libuvc/blob/master/src/stream.c#L1100)
* Merge pull request `#45 <https://github.com/ros-drivers/libuvc_ros/issues/45>`_ from mikaelarguedas/patch-2
  update to use non deprecated pluginlib macro
* update to use non deprecated pluginlib macro
* Merge pull request `#42 <https://github.com/ros-drivers/libuvc_ros/issues/42>`_ from mikaelarguedas/patch-1
  fix compiler warning
* fix compiler warning
  http://build.ros.org/view/Ldev/job/Ldev__libuvc_ros__ubuntu_xenial_amd64/3/warnings21Result/
* Contributors: Kei Okada, Mikael Arguedas

0.0.9 (2017-06-15)
------------------
* enable to compile with libuvc <= v0.0.5
* Contributors: Kei Okada

0.0.8 (2017-06-14)
------------------
* add ROS Orphaned Package Maintainers to maintainer tag (`#40 <https://github.com/ros-drivers/libuvc_ros/issues/40>`_)
* Implement missing index select behavior (`#27 <https://github.com/ros-drivers/libuvc_ros/issues/27>`_)
* Enable mjpeg support despite uvs_any2bgr shortcoming (`#26 <https://github.com/ros-drivers/libuvc_ros/issues/26>`_)
* [libuvc_camera/src/camera_driver.cpp] use frame's capture time for   timestamp of ros message instead of callback time (`#24 <https://github.com/ros-drivers/libuvc_ros/issues/24>`_)
* [libuvc_camera] support multiple video mode (`#22 <https://github.com/ros-drivers/libuvc_ros/issues/22>`_)
  * [libuvc_camera] add detail error message if no image format support
  * [libuvc_camera] support multiple video_mode
* add new parameters in cfg (`#21 <https://github.com/ros-drivers/libuvc_ros/issues/21>`_)
* Changed defaults: auto_exposure=True, auto_focus=aperture_priority
* Implemented AE priority, abs exposure/focus, autofocus, pantilt controls
* Contributors: Yuki Furuta, Josh Villbrandt, Kei Okada, Ken Tossell

0.0.7 (2014-03-06)
------------------
* Removed dependency on the deprecated driver_base package.
* Added more informative error messages in the case of uvc_open() failure

0.0.6 (2013-12-19)
------------------
* Install libuvc_camera_nodelet.xml

0.0.5 (2013-10-04)
------------------
* Fixed missing timestamp and frame_id in camera_info and image_raw
* Install camera_node into the package's bin path, not ros-global bin.

0.0.4 (2013-09-23)
------------------
* Added libuvc_ in libuvc_camera_nodelet

0.0.3 (2013-09-23)
------------------
* add an explicit dependency on generated files in CMake
  This will make sure that the generated headers are build before they are compiled.

0.0.2 (2013-08-12)
------------------
* libuvc_camera now uses the libuvc cmake config script
* Fixed PACKAGE reference.
* Removed roslib setup code from UVCCamera.cfg.

0.0.1 (2013-08-01)
------------------

0.0.0 (2013-06-27)
------------------
* fixed dependencies and nodelet export
* adding missing catkin files
* catkin build
* License blocks
* added parameter for timestamping method
* added video format param. set min=0 for unsigned int params
* Added libuvc_camera nodelet
* reset driver state after closing uvc context
* forgot to uvc_exit(ctx) -- driver now closes on sigint
* update manifests
* use camera_info_manager for a real CameraInfo
* initial use of auto-update controls
  The user can change AE mode, and the camera auto-updates
  absolute exposure time and white balance temperature, which
  are sent on to dynamic_reconfigure clients.
* fix rgb_frame_ leak
* beginning of driver with controls, dyn_reconf, caminfo
* added parameters ~vendor, ~product, ~serial_num
  ~vendor and ~product should be 0xhexadecimal. ~serial_num
  can be any string.
* added -luvc
* sample ROS driver using libuvc
