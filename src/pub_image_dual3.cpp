#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include "MvCameraControl.h"
#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <thread>
#include <mutex>  // 添加互斥锁的头文件

#define NIL (0)
#define MAX_IMAGE_DATA_SIZE (4 * 2048 * 2448)

bool PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo);

void CaptureAndPublish(image_transport::Publisher& pub, MV_CC_DEVICE_INFO* deviceInfo, std::mutex& mtx);

int main(int argc, char** argv) {
    ros::init(argc, argv, "pub_images");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub1 = it.advertise("/camera/right/image_raw", 10);
    image_transport::Publisher pub2 = it.advertise("/camera/left/image_raw", 10);

    int nRet = MV_OK;

    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

    nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (MV_OK != nRet) {
        printf("MV_CC_EnumDevices fail! nRet [%x]\n", nRet);
        return -1;
    }

    std::mutex mtx1; // 为每个相机创建一个互斥锁
    std::mutex mtx2;
    std::thread thread1(CaptureAndPublish, std::ref(pub1), stDeviceList.pDeviceInfo[0], std::ref(mtx1));
    std::thread thread2(CaptureAndPublish, std::ref(pub2), stDeviceList.pDeviceInfo[1], std::ref(mtx2));

    thread1.join();
    thread2.join();

    ROS_INFO("All camera threads finished.");
    return 0;
}

void CaptureAndPublish(image_transport::Publisher& pub, MV_CC_DEVICE_INFO* deviceInfo, std::mutex& mtx) {
    int nRet = MV_OK;
    void* handle = NULL;
    unsigned char* pData = NULL;
    unsigned char* pDataForBGR = NULL;

    nRet = MV_CC_CreateHandle(&handle, deviceInfo);
    if (MV_OK != nRet) {
        printf("MV_CC_CreateHandle fail! nRet [%x]\n", nRet);
        return;
    }

    nRet = MV_CC_OpenDevice(handle);
    if (MV_OK != nRet) {
        printf("MV_CC_OpenDevice fail! nRet [%x]\n", nRet);
        MV_CC_DestroyHandle(handle);
        return;
    }

    // Camera setup code (e.g., setting exposure, packet size, etc.)

    MVCC_INTVALUE stParam;
    memset(&stParam, 0, sizeof(MVCC_INTVALUE));
    nRet = MV_CC_GetIntValue(handle, "PayloadSize", &stParam);
    if (MV_OK != nRet) {
        printf("Get PayloadSize fail! nRet [0x%x]\n", nRet);
        return;
    }

    // @TODO:设置参数，具体看手册《Machine Vision Camera SDK (C)_Developer Guide_V3.2.0_CH》
    nRet = MV_CC_SetFloatValue(handle, "ExposureTime", 10000);
    if(MV_OK != nRet){
        printf("MV_CC_SetEnumValue ExposureTime fail! nRet [%x]\n", nRet);
        return;
    }
    else
        printf("MV_CC_SetEnumValue ExposureTime success\n");

    nRet = MV_CC_SetFloatValue(handle, "Gain", 15.0);
    if(MV_OK != nRet){
        printf("MV_CC_SetEnumValue Gain fail! nRet [%x]\n", nRet);
        return;
    }
    else
        printf("MV_CC_SetEnumValue Gain success\n");

    // 设置触发模式为off
    // set trigger mode as off
    nRet = MV_CC_SetEnumValue(handle, "TriggerMode", 0);
    if (MV_OK != nRet)
    {
        printf("MV_CC_SetTriggerMode fail! nRet [%x]\n", nRet);
        return;
    }

    nRet = MV_CC_SetEnumValue(handle, "AcquisitionMode", 2);
    if (MV_OK != nRet)
    {
        printf("Cam[%d]: AcquisitionMode fail! nRet \n", nRet);
    }

    nRet = MV_CC_SetFloatValue(handle, "AcquisitionFrameRate", 30.0);// @TODO 帧率
    if(MV_OK != nRet){
        printf("MV_CC_SetEnumValue AcquisitionFrameRate fail! nRet [%x]\n", nRet);
        return;
    }
    else
        printf("MV_CC_SetEnumValue ResultingFrameRate success\n");

    // 开始取流
    // start grab image
    nRet = MV_CC_StartGrabbing(handle);
    if (MV_OK != nRet) {
        printf("MV_CC_StartGrabbing fail! nRet [%x]\n", nRet);
        return;
    }

    MV_FRAME_OUT_INFO_EX stImageInfo = { 0 };
    memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
    pData = (unsigned char*)malloc(sizeof(unsigned char) * stParam.nCurValue);
    if (NULL == pData) {
        return;
    }
    unsigned int nDataSize = stParam.nCurValue;

    ros::Rate loop_rate(10);

    while (ros::ok()) {
        nRet = MV_CC_GetOneFrameTimeout(handle, pData, nDataSize, &stImageInfo, 1000);
        if (MV_OK == nRet) {
            pDataForBGR = (unsigned char*)malloc(MAX_IMAGE_DATA_SIZE);
            if (NULL == pDataForBGR) {
                return;
            }

            MV_CC_PIXEL_CONVERT_PARAM stConvertParam = { 0 };
            stConvertParam.nWidth = stImageInfo.nWidth;
            stConvertParam.nHeight = stImageInfo.nHeight;
            stConvertParam.pSrcData = pData;
            stConvertParam.nSrcDataLen = stImageInfo.nFrameLen;
            stConvertParam.enSrcPixelType = stImageInfo.enPixelType;
            stConvertParam.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
            stConvertParam.pDstBuffer = pDataForBGR;
            stConvertParam.nDstBufferSize = MAX_IMAGE_DATA_SIZE;

            // 添加互斥锁以确保图像数据的安全访问
            std::lock_guard<std::mutex> lock(mtx);

            nRet = MV_CC_ConvertPixelType(handle, &stConvertParam);
            if (MV_OK != nRet) {
                printf("MV_CC_ConvertPixelType fail! nRet [%x]\n", nRet);
                return;
            }

            cv::Mat frame = cv::Mat(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC3, pDataForBGR).clone();
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
            pub.publish(msg);
            loop_rate.sleep();
        }
    }

    nRet = MV_CC_StopGrabbing(handle);
    if (MV_OK != nRet) {
        printf("MV_CC_StopGrabbing fail! nRet [%x]\n", nRet);
        return;
    }

    nRet = MV_CC_CloseDevice(handle);
    if (MV_OK != nRet) {
        printf("MV_CC_CloseDevice fail! nRet [%x]\n", nRet);
        return;
    }

    nRet = MV_CC_DestroyHandle(handle);
    if (MV_OK != nRet) {
        printf("MV_CC_DestroyHandle fail! nRet [%x]\n", nRet);
        return;
    }

    printf("Thread for camera finished.\n");
}

bool PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo) {
    if (NULL == pstMVDevInfo) {
        printf("The Pointer of pstMVDevInfo is NULL!\n");
        return false;
    }
    if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE) {
        int nIp1 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
        int nIp2 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
        int nIp3 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
        int nIp4 = (pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);

        printf("Device Model Name: %s\n", pstMVDevInfo->SpecialInfo.stGigEInfo.chModelName);
        printf("CurrentIp: %d.%d.%d.%d\n", nIp1, nIp2, nIp3, nIp4);
        printf("UserDefinedName: %s\n\n", pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
    }
    else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE) {
        printf("Device Model Name: %s\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chModelName);
        printf("UserDefinedName: %s\n\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
    }
    else {
        printf("Not supported.\n");
    }

    return true;
}
