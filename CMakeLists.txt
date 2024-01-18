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
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS} /usr/local/opencv320/include)
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
add_executable(pub_image src/pub_image.cpp)
add_executable(pub_image_left src/pub_image_left.cpp)
add_executable(pub_image_right src/pub_image_right.cpp)
add_executable(GrabMultipleCamera src/GrabMultipleCamera.cpp)
add_executable(Trigger_Image src/Trigger_Image.cpp)
add_executable(pub_image_dual src/pub_image_dual.cpp)#似乎有问题，先不用
add_executable(pub_image_dual2 src/pub_image_dual2.cpp)#暂时用这个
add_executable(pub_image_dual3 src/pub_image_dual3.cpp)#加入互斥锁，不过似乎不是这个的问题。最后通过降低帧率解决的，之前帧率的设置似乎有问题
add_executable(pub_image_dual_trigger src/pub_image_dual_trigger.cpp)#加入互斥锁，不过似乎不是这个的问题。最后通过降低帧率解决的，之前帧率的设置似乎有问题
target_link_libraries(pub_image
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${cv_bridge_LIBRARIES}
     #GCBase_gcc421_v3_0
   MvCameraControl
   #GenApi_gcc421_v3_0
   MVGigEVisionSDK
   #log4cpp_gcc421_v3_0
   MVRender
   #Log_gcc421_v3_0
   MvUsb3vTL
   #MathParser_gcc421_v3_0
   #NodeMapData_gcc421_v3_0
   MediaProcess
   #XmlParser_gcc421_v3_0
   X11
  #GCBase_gcc48_v3_0
   #MvCameraControl
   #GenApi_gcc48_v3_0
   #MVGigEVisionSDK
   #log4cpp_gcc48_v3_0
   #MVRender
   #Log_gcc48_v3_0
   #MvUsb3vTL
   #MathParser_gcc48_v3_0
   #NodeMapData_gcc48_v3_0
   #MediaProcess
   #XmlParser_gcc48_v3_0
   #X11
)

target_link_libraries(pub_image_left
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        ${cv_bridge_LIBRARIES}
        #GCBase_gcc421_v3_0
        MvCameraControl
        #GenApi_gcc421_v3_0
        MVGigEVisionSDK
        #log4cpp_gcc421_v3_0
        MVRender
        #Log_gcc421_v3_0
        MvUsb3vTL
        #MathParser_gcc421_v3_0
        #NodeMapData_gcc421_v3_0
        MediaProcess
        #XmlParser_gcc421_v3_0
        X11
        )

target_link_libraries(pub_image_right
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        ${cv_bridge_LIBRARIES}
        #GCBase_gcc421_v3_0
        MvCameraControl
        #GenApi_gcc421_v3_0
        MVGigEVisionSDK
        #log4cpp_gcc421_v3_0
        MVRender
        #Log_gcc421_v3_0
        MvUsb3vTL
        #MathParser_gcc421_v3_0
        #NodeMapData_gcc421_v3_0
        MediaProcess
        #XmlParser_gcc421_v3_0
        X11
        )

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

target_link_libraries(pub_image_dual
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        ${cv_bridge_LIBRARIES}
        #GCBase_gcc421_v3_0
        MvCameraControl
        #GenApi_gcc421_v3_0
        MVGigEVisionSDK
        #log4cpp_gcc421_v3_0
        MVRender
        #Log_gcc421_v3_0
        MvUsb3vTL
        #MathParser_gcc421_v3_0
        #NodeMapData_gcc421_v3_0
        MediaProcess
        #XmlParser_gcc421_v3_0
        X11
        )

target_link_libraries(pub_image_dual2
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        ${cv_bridge_LIBRARIES}
        #GCBase_gcc421_v3_0
        MvCameraControl
        #GenApi_gcc421_v3_0
        MVGigEVisionSDK
        #log4cpp_gcc421_v3_0
        MVRender
        #Log_gcc421_v3_0
        MvUsb3vTL
        #MathParser_gcc421_v3_0
        #NodeMapData_gcc421_v3_0
        MediaProcess
        #XmlParser_gcc421_v3_0
        X11
        )

target_link_libraries(pub_image_dual3
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        ${cv_bridge_LIBRARIES}
        #GCBase_gcc421_v3_0
        MvCameraControl
        #GenApi_gcc421_v3_0
        MVGigEVisionSDK
        #log4cpp_gcc421_v3_0
        MVRender
        #Log_gcc421_v3_0
        MvUsb3vTL
        #MathParser_gcc421_v3_0
        #NodeMapData_gcc421_v3_0
        MediaProcess
        #XmlParser_gcc421_v3_0
        X11
        )

target_link_libraries(pub_image_dual_trigger
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        ${cv_bridge_LIBRARIES}
        #GCBase_gcc421_v3_0
        MvCameraControl
        #GenApi_gcc421_v3_0
        MVGigEVisionSDK
        #log4cpp_gcc421_v3_0
        MVRender
        #Log_gcc421_v3_0
        MvUsb3vTL
        #MathParser_gcc421_v3_0
        #NodeMapData_gcc421_v3_0
        MediaProcess
        #XmlParser_gcc421_v3_0
        X11
        )