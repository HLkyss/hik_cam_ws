cmake_minimum_required(VERSION 3.0.2)
project(mvs_image_ros)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")
#调试模式
SET(CMAKE_BUILD_TYPE "Debug")
#find_package(OpenCV REQUIRED)

set(cv_bridge_DIR /home/hl/Downloads/vision_opencv-noetic/cv_bridge/install/install320/usr/local/share/cv_bridge/cmake)  #在find_package前面
find_package(cv_bridge)
set(OpenCV_DIR "/usr/local/opencv320")
set(OpenCV_LIBS "/usr/local/opencv320/lib")
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS} /usr/local/opencv320/include /home/hl/project/camera/hik_cam_ros/hik_4_ws/src/HikRobot-Camera-ROS-main/src/driver/camera/inc)
find_package(OpenCV 3.2)

find_package(catkin REQUIRED COMPONENTS
  #cv_bridge
  roscpp
  sensor_msgs
  image_transport
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mvs_image_ros
  CATKIN_DEPENDS cv_bridge roscpp sensor_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  /opt/MVS/include
  ${cv_bridge_INCLUDE_DIRS}
)
LINK_DIRECTORIES(
  /opt/MVS/lib/64/
)

# link_directories(${OpenCV_LIBRARY_DIRS})
link_directories(/home/hl/project/camera/hik_cam_ros/hik_4_ws/src/HikRobot-Camera-ROS-main/src/driver/camera/build/)

add_executable(GrabMultipleCamera src/GrabMultipleCamera.cpp)
add_executable(Trigger_Image src/Trigger_Image.cpp)
add_executable(pub_image_dual3 src/pub_image_dual3.cpp)#加入互斥锁，不过似乎不是这个的问题。最后通过降低帧率解决的，之前帧率的设置似乎有问题
add_executable(pub_image_dual_trigger src/pub_image_dual_trigger.cpp)#加入互斥锁，不过似乎不是这个的问题。最后通过降低帧率解决的，之前帧率的设置似乎有问题
add_executable(pub_image_dual_trigger2 src/pub_image_dual_trigger2.cpp)#加入互斥锁，不过似乎不是这个的问题。最后通过降低帧率解决的，之前帧率的设置似乎有问题
target_link_libraries(GrabMultipleCamera
        ${OpenCV_LIBS}
        MvCameraControl
        MVGigEVisionSDK
        MVRender
        MvUsb3vTL
        MediaProcess
        X11
        )
target_link_libraries(Trigger_Image
        ${OpenCV_LIBS}
        MvCameraControl
        MVGigEVisionSDK
        MVRender
        MvUsb3vTL
        MediaProcess
        X11
        )
set(COMMON_LIBRARIES
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        ${cv_bridge_LIBRARIES}
        MvCameraControl
        MVGigEVisionSDK
        MVRender
        MvUsb3vTL
        MediaProcess
        X11
        camera
        )
target_link_libraries(pub_image_dual3
        ${COMMON_LIBRARIES}
        )
target_link_libraries(pub_image_dual_trigger
        ${COMMON_LIBRARIES}
        )
target_link_libraries(pub_image_dual_trigger2
        ${COMMON_LIBRARIES}
        )

##################################################

