^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package libuvc_camera
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
