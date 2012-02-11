#pragma once

#include <libuvc/libuvc.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/camera_publisher.h>
#include <dynamic_reconfigure/server.h>
#include <boost/thread/mutex.hpp>

#include <libuvc_camera/UVCCameraConfig.h>

namespace libuvc_camera {

class CameraDriver {
public:
  CameraDriver(ros::NodeHandle nh, ros::NodeHandle priv_nh);
  ~CameraDriver();

  bool Start();
  void Stop();

private:
  enum State {
    kInitial = 0,
    kStopped = 1,
    kRunning = 2,
  };

  void OpenCamera(UVCCameraConfig &new_config);
  void CloseCamera();

  // Accept a reconfigure request from a client
  void ReconfigureCallback(UVCCameraConfig &config, uint32_t level);
  // Accept changes in values of automatically updated controls
  void AutoControlsCallback();
  static void AutoControlsCallbackAdapter(void *ptr);
  // Accept a new image frame from the camera
  void ImageCallback(uvc_frame_t *frame);
  static void ImageCallbackAdapter(uvc_frame_t *frame, void *ptr);

  ros::NodeHandle nh_, priv_nh_;

  State state_;
  boost::mutex mutex_;

  uvc_context_t *ctx_;
  uvc_device_t *dev_;
  uvc_device_handle_t *devh_;
  uvc_frame_t *rgb_frame_;

  image_transport::ImageTransport it_;
  image_transport::CameraPublisher cam_pub_;

  dynamic_reconfigure::Server<UVCCameraConfig> config_server_;
  UVCCameraConfig config_;
};

};
