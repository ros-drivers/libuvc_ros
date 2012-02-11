#include <ros/ros.h>

#include "libuvc_camera/camera_driver.h"

int main (int argc, char **argv) {
  ros::init(argc, argv, "libuvc_camera");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");

  libuvc_camera::CameraDriver driver(nh, priv_nh);

  if (!driver.Start())
    return -1;

  ros::spin();

  driver.Stop();

  return 0;
}
