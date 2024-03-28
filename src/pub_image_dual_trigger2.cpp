#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "HIK_camera.h"
#include "cv_bridge/cv_bridge.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "pub_images");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    // 实例化两个相机对象
    ly::HIK_camera camera1(0);
    ly::HIK_camera camera2(1);

    // 为每个相机创建一个发布者
    image_transport::Publisher pub_camera1 = it.advertise("/camera/right/image_raw", 10);
    image_transport::Publisher pub_camera2 = it.advertise("/camera/left/image_raw", 10);

    ros::Rate loop_rate(30); // 设置帧率
    while (nh.ok()) {
        cv::Mat frame1 = camera1.getFrame();
        cv::Mat frame2 = camera2.getFrame();

        if (!frame1.empty()) {
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame1).toImageMsg();
            pub_camera1.publish(msg);
        }

        if (!frame2.empty()) {
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame2).toImageMsg();
            pub_camera2.publish(msg);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
