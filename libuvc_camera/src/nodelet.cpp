#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "libuvc_camera/camera_driver.h"

namespace libuvc_camera {

class CameraNodelet : public nodelet::Nodelet {
public:
  CameraNodelet() : running_(false) {}
  ~CameraNodelet();

private:
  virtual void onInit();

  volatile bool running_;
  boost::shared_ptr<CameraDriver> driver_;
};

CameraNodelet::~CameraNodelet() {
  if (running_) {
    driver_->Stop();
  }
}

void CameraNodelet::onInit() {
  ros::NodeHandle nh(getNodeHandle());
  ros::NodeHandle priv_nh(getPrivateNodeHandle());

  driver_.reset(new CameraDriver(nh, priv_nh));
  if (driver_->Start()) {
    running_ = true;
  } else {
    NODELET_ERROR("Unable to open camera.");
    driver_.reset();
  }
}

};

// Register this plugin with pluginlib.
//
// parameters are: package, class name, class type, base class type
PLUGINLIB_DECLARE_CLASS(libuvc_camera, driver,
                        libuvc_camera::CameraNodelet, nodelet::Nodelet);
