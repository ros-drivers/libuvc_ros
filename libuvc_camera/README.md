## Overview

This package provides a ROS interface for digital cameras meeting the
USB Video Class standard (UVC) using
[libuvc](http://github.com/ktossell/libuvc/). Most webcams are
UVC-compliant.

## Limitations

Some controls are missing, though the most common ones (focus, exposure
and others) are supported. There is no support for UVC extension units,
which are sometimes used to implement behavior such as pan-tilt and
zoom.

## Features

Current features include:

- Dynamic reconfiguration and monitoring of the camera's control
  settings
- Camera calibration
  ([camera_info_manager](https://index.ros.org/p/camera_info_manager))
- Image pipeline integration
  ([image_transport](https://index.ros.org/p/image_transport))

## Usage

### Permissions

Under Linux, the user that runs camera_node must have write permissions
to the `/dev/bus/usb/...` device that corresponds to the camera. You may
run the node as root:



```
$ sudo -E rosrun libuvc_camera camera_node vendor:=...
```

 This command gives information on the webcam's
vendor and product ID's:



```
lsusb -v
```

 However, use of *udev* rules is recommended. In
`/etc/udev/rules.d/99-uvc.rules`, to give every user camera access:



```
# UVC cameras
SUBSYSTEMS=="usb", ENV{DEVTYPE}=="usb_device", ATTRS{idVendor}=="046d", ATTRS{idProduct}=="08cc", MODE="0666"
# ^ Change the vendor and product IDs to match your camera.
```

 To give only one user access:



```
# UVC cameras
SUBSYSTEMS=="usb", ENV{DEVTYPE}=="usb_device", ATTRS{idVendor}=="046d", ATTRS{idProduct}=="08cc", OWNER="myuser"
# ^ Change the owner and the vendor and product IDs to match your camera.
```

 Alternatively, set `GROUP="mygroup"` to grant
access for any user in that group.

The following rule works with any Logitech camera:



```
#UVC cameras
SUBSYSTEMS=="usb", ATTRS{idVendor}=="046d", MODE="0666"
```

 These settings will be applied the next time
the camera is connected.

**REMINDER: For Ubuntu 12.04 and above the udev rules syntax changed!
Replace SYSFS{\...} with ATTRS{\...}**

Or, run the following before starting the camera. It gets access to the
USB device:



```
sudo chmod o+w /dev/bus/usb/001/024 <-- where 024 is port# from the error code
```



### Running the driver

You may need to disable your operating system's builtin USB video or
audio drivers. On Linux, the `snd-usb-audio` and `uvcvideo` modules
conflict with libuvc. Try unloading them with `sudo rmmod
snd-usb-audio; sudo rmmod uvcvideo` and consider blacklisting them \--
e.g., add the lines `blacklist uvcvideo` and `blacklist
snd-usb-audio` to an `/etc/modprobe.d/uvc.conf` file. (Applications
that don't use libuvc will be unable to stream from the camera.)

Get started with the following launch file. If you need assistance with
your webcam's supported resolutions and frame rates, try:



```
v4l2-ctl --list-formats-ext
```

 

```
<launch>
  <group ns="camera">
    <node pkg="libuvc_camera" type="camera_node" name="mycam">
      <!-- Parameters used to find the camera -->
      <param name="vendor" value="0x0"/>
      <param name="product" value="0x0"/>
      <param name="serial" value=""/>
      <!-- If the above parameters aren't unique, choose the first match: -->
      <param name="index" value="0"/>

      <!-- Image size and type -->
      <param name="width" value="640"/>
      <param name="height" value="480"/>
      <!-- choose whichever uncompressed format the camera supports: -->
      <param name="video_mode" value="uncompressed"/> <!-- or yuyv/nv12/mjpeg -->
      <param name="frame_rate" value="15"/>

      <param name="timestamp_method" value="start"/> <!-- start of frame -->
      <param name="camera_info_url" value="file:///tmp/cam.yaml"/>

      <param name="auto_exposure" value="3"/> <!-- use aperture_priority auto exposure -->
      <param name="auto_white_balance" value="false"/>
    </node>
  </group>
</launch>
```


<a id="ROS_API"></a>

## ROS API

<a id="camera_node"></a>

### camera_node

UVC webcam driver.

#### Published Topics

- `image_raw`
  ([sensor_msgs/msg/Image](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/Image.html)):
  A stream of images from the camera.
- `camera_info`
  ([sensor_msgs/msg/CameraInfo](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/CameraInfo.html)):
  Camera intrinsics for images published on `camera/image_raw` with
  matching time stamps and frame IDs. If !CameraInfo calibration is not
  available or is incompatible with the current `video_mode`,
  uncalibrated data will be provided instead.

#### Services

- `set_camera_info`
  ([sensor_msgs/srv/SetCameraInfo](https://docs.ros.org/en/rolling/p/sensor_msgs/srv/SetCameraInfo.html)):
  Set the camera's calibration.

#### Parameters

##### Dynamically Reconfigurable Parameters

See the
[dynamic_reconfigure](https://index.ros.org/p/dynamic_reconfigure)
package for details on dynamically reconfigurable parameters.

- `~vendor` (`str`): Vendor ID, hex digits including '0x' ex:
  '0x046d'(use camera of any vendor if null).
- `~product` (`str`): Product ID, hex digits including '0x' ex:
  '0x0809' (use camera of any model if null).
- `~serial` (`str`): Serial number, arbitrary string (use camera of any
  serial number if null).
- `~index` (`int`, default: `0`): Index into the list of cameras that
  match the above parameters. Range: 0 to 2147483647
- `~width` (`int`, default: `640`): Image width. Range: 0 to 2147483647
- `~height` (`int`, default: `480`): Image height. Range: 0 to
  2147483647
- `~video_mode` (`str`, default: `uncompressed`): Format of video stream
  from camera. Possible values are: uncompressed (uncompressed): Use any
  uncompressed format, yuyv (yuyv): YUYV, uyvy (uyvy): UYVY, rgb (rgb):
  RGB, jpeg (jpeg): JPEG/MJPEG
- `~frame_rate` (`double`, default: `15.0`): Camera speed, frames per
  second. Range: 0.1 to 1000.0
- `~timestamp_method` (`str`, default: `start`): Method for determining
  the timestamp. Possible values are: PubTime (pub): Time of
  publication, FrameStartTime (start): Time when raw frame capture
  began, FrameStopTime (stop): Time when raw frame capture ended,
  HostReceiptTime (hostrcpt): Time when camera-to-host transfer
  completed
- `~frame_id` (`str`, default: `camera`): ROS tf frame of reference,
  resolved with tf_prefix unless absolute.
- `~camera_info_url` (`str`): Path to camera calibration file.
- `~scanning_mode` (`int`, default: `0`): Scanning mode. Possible values
  are: Interlaced (0): , Progressive (1):
- `~auto_exposure` (`int`, default: `1`): Auto exposure mode. Possible
  values are: Manual (0): Manual exposure, manual iris, Auto (1): Auto
  exposure, auto iris, Shutter_Priority (2): manual exposure, auto iris,
  Aperture_Priority (3): auto exposure, manual iris
- `~auto_exposure_priority` (`int`, default: `0`): In auto mode or
  shutter priority mode, allow the device to vary frame rate. Range: 0
  to 1
- `~exposure_absolute` (`double`, default: `0.0`): Length of exposure,
  seconds. Range: 0.0001 to 10.0
- `~iris_absolute` (`double`, default: `0.0`): Aperture, f. Range: 0.0
  to 655.35
- `~auto_focus` (`bool`, default: `False`): Maintain focus
  automatically.
- `~focus_absolute` (`int`, default: `0`): Absolute focal distance,
  millimeters. Range: 0 to 65536
- `~pan_absolute` (`int`, default: `0`): Pan (clockwise), arc seconds.
  Range: -648000 to 648000
- `~tilt_absolute` (`int`, default: `0`): Tilt (up), arc seconds. Range:
  -648000 to 648000
- `~roll_absolute` (`int`, default: `0`): Roll (clockwise), degrees.
  Range: -180 to 180
- `~privacy` (`bool`, default: `False`): Image capture disabled.
- `~backlight_compensation` (`int`, default: `0`): Backlight
  compensation, device-dependent (zero for none, increasing compensation
  above zero). Range: 0 to 65536
- `~brightness` (`int`, default: `0`): Brightness, device dependent.
  Range: -32768 to 32767
- `~contrast` (`int`, default: `0`): Contrast, device dependent. Range:
  -32768 to 32767
- `~gain` (`int`, default: `0`): Gain, device dependent. Range: 0 to
  65536
- `~power_line_frequency` (`int`, default: `0`): Power line frequency
  anti-flicker processing. Possible values are: Disabled (0): Disabled,
  Freq_50 (1): 50 Hz, Freq_60 (1): 60 Hz
- `~auto_hue` (`bool`, default: `False`): Automatic hue control.
- `~hue` (`double`, default: `0.0`): Hue, degrees. Range: -180.0 to
  180.0
- `~saturation` (`int`, default: `0`): Saturation, device dependent
  (zero for grayscale). Range: 0 to 65536
- `~sharpness` (`int`, default: `0`): Image sharpness, device dependent.
  Range: 0 to 65536
- `~gamma` (`double`, default: `1.0`): Gamma. Range: 0.01 to 5.0
- `~auto_white_balance` (`bool`, default: `False`): Automatic white
  balance.
- `~white_balance_temperature` (`int`, default: `0`): White balance
  temperature, degrees. Range: 0 to 65536
- `~white_balance_BU` (`double`, default: `0.0`): Blue or U component of
  white balance, device-dependent. Range: 0.0 to 65536.0
- `~white_balance_RV` (`double`, default: `0.0`): Red or V component of
  white balance, device-dependent. Range: 0.0 to 65536.0
