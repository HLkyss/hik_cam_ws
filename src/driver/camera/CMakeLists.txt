find_package(Threads REQUIRED)
#find_package(OpenCV REQUIRED)
set(OpenCV_LIBS "/usr/local/opencv320/lib")
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS} /usr/local/opencv320/include /home/hl/project/camera/hik_cam_ros/hik_4_ws/src/HikRobot-Camera-ROS-main/src/driver/camera/inc)
find_package(OpenCV 3.2)

include_directories(
        inc
        #../../tool/inc
        ${PROJECT_SOURCE_DIR}/driver/camera/lib/hik_include
		#${PROJECT_SOURCE_DIR}/driver/camera/lib/dahen_include
        ${Boost_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
		/opt/MVS/include

)

add_library(
        camera
        src/camera.cpp
        src/HIK_camera.cpp
		#src/DAHEN_camera.cpp
)
target_link_libraries(
        camera
        ${OpenCV_LIBS}
		#${PROJECT_SOURCE_DIR}/driver/camera/lib/dahen_lib/libgxiapi.so
		${PROJECT_SOURCE_DIR}/driver/camera/lib/hik_lib/pc_64/libMvCameraControl.so
        -lpthread
)
