#include "libuvc_camera/camera_driver.h"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/camera_publisher.h>
#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure/SensorLevels.h>
#include <libuvc/libuvc.h>

namespace libuvc_camera {

CameraDriver::CameraDriver(ros::NodeHandle nh, ros::NodeHandle priv_nh)
  : nh_(nh), priv_nh_(priv_nh),
    state_(kInitial),
    ctx_(NULL), dev_(NULL), devh_(NULL), rgb_frame_(NULL),
    it_(nh_),
    config_server_(priv_nh_) {
  cam_pub_ = it_.advertiseCamera("image_raw", 1, false);
}

CameraDriver::~CameraDriver() {
  if (rgb_frame_)
    uvc_free_frame(rgb_frame_);

  if (ctx_)
    uvc_exit(ctx_);  // Destroys dev_, devh_, etc.
}

bool CameraDriver::Start() {
  assert(state_ == kInitial);

  uvc_error_t err;

  err = uvc_init(&ctx_, NULL);

  if (err != UVC_SUCCESS) {
    uvc_perror(err, "ERROR: uvc_init");
    return false;
  }

  state_ = kStopped;

  config_server_.setCallback(boost::bind(&CameraDriver::ReconfigureCallback, this, _1, _2));

  return state_ == kRunning;
}

void CameraDriver::Stop() {
  boost::mutex::scoped_lock(mutex_);

  if (state_ == kRunning) {
    CloseCamera();
    state_ = kInitial;
  }
}

void CameraDriver::ReconfigureCallback(UVCCameraConfig &new_config, uint32_t level) {
  boost::mutex::scoped_lock(mutex_);

  if (level & dynamic_reconfigure::SensorLevels::RECONFIGURE_CLOSE) {
    if (state_ == kRunning)
      CloseCamera();
  }

  if (state_ == kStopped) {
    OpenCamera(new_config);
  }

  if (state_ == kRunning) {
    // TODO: scanning_mode
    // TODO: auto_exposure
    // TODO: auto_exposure_priority
    // TODO: exposure_absolute
    // TODO: iris_absolute
    // TODO: auto_focus
    // TODO: focus_absolute
    // TODO: pan_absolute
    // TODO: tilt_absolute
    // TODO: roll_absolute
    // TODO: privacy
    // TODO: backlight_compensation
    // TODO: brightness
    // TODO: contrast
    // TODO: gain
    // TODO: power_line_frequency
    // TODO: auto_hue
    // TODO: saturation
    // TODO: sharpness
    // TODO: gamma
    // TODO: auto_white_balance
    // TODO: white_balance_temperature
    // TODO: white_balance_BU
    // TODO: white_balance_RV
  }

  config_ = new_config;
}

void CameraDriver::ImageCallback(uvc_frame_t *frame) {
  boost::mutex::scoped_lock(mutex_);

  assert(state_ == kRunning);
  assert(rgb_frame_);

  uvc_error_t conv_ret = uvc_any2rgb(frame, rgb_frame_);

  if (conv_ret != UVC_SUCCESS) {
    uvc_perror(conv_ret, "Couldn't convert frame to RGB");
    return;
  }

  sensor_msgs::Image image;
  image.width = config_.width;
  image.height = config_.height;
  image.encoding = "rgb8";
  image.step = image.width * 3;
  image.data.resize(image.step * image.height);
  memcpy(&(image.data[0]), rgb_frame_->data, rgb_frame_->data_bytes);

  cam_pub_.publish(image, sensor_msgs::CameraInfo());
}

/* static */ void CameraDriver::ImageCallbackAdapter(uvc_frame_t *frame, void *ptr) {
  CameraDriver *driver = static_cast<CameraDriver*>(ptr);

  driver->ImageCallback(frame);
}

void CameraDriver::AutoControlsCallback() {
  boost::mutex::scoped_lock(mutex_);

}

/* static */ void CameraDriver::AutoControlsCallbackAdapter(void *ptr) {
  CameraDriver *driver = static_cast<CameraDriver*>(ptr);

  driver->AutoControlsCallback();
}

void CameraDriver::OpenCamera(UVCCameraConfig &new_config) {
  assert(state_ == kStopped);

  int vendor_id = strtol(new_config.vendor.c_str(), NULL, 0);
  int product_id = strtol(new_config.product.c_str(), NULL, 0);

  ROS_INFO("Opening camera with vendor=0x%x, product=0x%x, serial=\"%s\", index=%d",
           vendor_id, product_id, new_config.serial.c_str(), new_config.index);

  uvc_error_t find_err = uvc_find_device(
    ctx_, &dev_,
    vendor_id,
    product_id,
    new_config.serial.empty() ? NULL : new_config.serial.c_str());

  // TODO: index

  if (find_err != UVC_SUCCESS) {
    uvc_perror(find_err, "uvc_find_device");
    return;
  }

  uvc_error_t open_err = uvc_open(dev_, &devh_);
  if (open_err != UVC_SUCCESS) {
    uvc_perror(open_err, "uvc_open");
    uvc_unref_device(dev_);
    return;
  }

  uvc_stream_ctrl_t ctrl;
  uvc_error_t mode_err = uvc_get_stream_ctrl_format_size(
    devh_, &ctrl,
    UVC_COLOR_FORMAT_UNCOMPRESSED,
    new_config.width, new_config.height,
    new_config.frame_rate);

  if (mode_err != UVC_SUCCESS) {
    uvc_perror(mode_err, "uvc_get_stream_ctrl_format_size");
    uvc_close(devh_);
    uvc_unref_device(dev_);
    return;
  }

  uvc_error_t stream_err = uvc_start_iso_streaming(devh_, &ctrl, &CameraDriver::ImageCallbackAdapter, this);

  if (stream_err != UVC_SUCCESS) {
    uvc_perror(stream_err, "uvc_start_iso_streaming");
    uvc_close(devh_);
    uvc_unref_device(dev_);
    return;
  }

  if (rgb_frame_)
    uvc_free_frame(rgb_frame_);

  rgb_frame_ = uvc_allocate_frame(new_config.width * new_config.height * 3);
  assert(rgb_frame_);

  state_ = kRunning;
}

void CameraDriver::CloseCamera() {
  assert(state_ == kRunning);

  uvc_close(devh_);
  devh_ = NULL;

  uvc_unref_device(dev_);
  dev_ = NULL;

  state_ = kStopped;
}

};
