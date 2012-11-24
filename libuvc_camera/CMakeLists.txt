# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(libuvc_camera)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS libuvc)

#find_package(libuvc REQUIRED)
catkin_package(DEPENDS roscpp camera_info_manager driver_base dynamic_reconfigure image_transport nodelet sensor_msgs libuvc
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME})

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LINK_DIRS})

find_package(catkin REQUIRED dynamic_reconfigure)
generate_dynamic_reconfigure_options(cfg/UVCCamera.cfg)

find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})

#find_package(libuvc REQUIRED)
include_directories(${libuvc_INCLUDE_DIRS})

add_executable(camera_node src/main.cpp src/camera_driver.cpp)
target_link_libraries(camera_node uvc ${Boost_LIBRARIES})

add_library(libuvc_camera_nodelet src/nodelet.cpp src/camera_driver.cpp)
target_link_libraries(libuvc_camera_nodelet uvc ${Boost_LIBRARIES})
