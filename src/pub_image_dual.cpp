//大部分代码来自于官方SDK的例程GrabImage.cpp、ImageProcess.cpp
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

#define NIL (0)
#define MAX_IMAGE_DATA_SIZE (4 * 2048 * 2448)

#define CAMERA_NUM             2

bool PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo);

// 全局发布者变量
image_transport::Publisher pub1;
image_transport::Publisher pub2;

static void* WorkThread(void* pUser)
{
    int nRet = MV_OK;

    MVCC_STRINGVALUE stStringValue = {0};
    char camSerialNumber[256] = {0};
    nRet = MV_CC_GetStringValue(pUser, "DeviceSerialNumber", &stStringValue);
    if (MV_OK == nRet)
    {
        memcpy(camSerialNumber, stStringValue.chCurValue, sizeof(stStringValue.chCurValue));
    }
    else
    {
        printf("Get DeviceUserID Failed! nRet = [%x]\n", nRet);
    }

    // ch:获取数据包大小 | en:Get payload size
    MVCC_INTVALUE stParam;
    memset(&stParam, 0, sizeof(MVCC_INTVALUE));
    nRet = MV_CC_GetIntValue(pUser, "PayloadSize", &stParam);
    if (MV_OK != nRet)
    {
        printf("Get PayloadSize fail! nRet [0x%x]\n", nRet);
        return NULL;
    }

    MV_FRAME_OUT_INFO_EX stImageInfo = {0};
    memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
    unsigned char * pData = (unsigned char *)malloc(sizeof(unsigned char) * stParam.nCurValue);
    if (NULL == pData)
    {
        return NULL;
    }
    unsigned int nDataSize = stParam.nCurValue;

    while(1)//这里换成ros::Rate loop_rate(10);
    {
        nRet = MV_CC_GetOneFrameTimeout(pUser, pData, nDataSize, &stImageInfo, 1000);
        if (nRet == MV_OK)
        {
            unsigned char *pDataForBGR = NULL;
            pDataForBGR = (unsigned char *)malloc(MAX_IMAGE_DATA_SIZE);;
            if (NULL == pDataForBGR)
            {
                break;
            }
            // 像素格式转换
            // convert pixel format
            MV_CC_PIXEL_CONVERT_PARAM stConvertParam = {0};
            // 从上到下依次是：图像宽，图像高，输入数据缓存，输入数据大小，源像素格式，
            // 目标像素格式，输出数据缓存，提供的输出缓冲区大小
            // Top to bottom are：image width, image height, input data buffer, input data size, source pixel format,
            // destination pixel format, output data buffer, provided output buffer size
            stConvertParam.nWidth = stImageInfo.nWidth;
            stConvertParam.nHeight = stImageInfo.nHeight;
            stConvertParam.pSrcData = pData;
            stConvertParam.nSrcDataLen = stImageInfo.nFrameLen;
            stConvertParam.enSrcPixelType = stImageInfo.enPixelType;
            stConvertParam.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
            stConvertParam.pDstBuffer = pDataForBGR;
            stConvertParam.nDstBufferSize = MAX_IMAGE_DATA_SIZE;
            nRet = MV_CC_ConvertPixelType(pUser, &stConvertParam);
            if (MV_OK != nRet)
            {
                printf("MV_CC_ConvertPixelType fail! nRet [%x]\n", nRet);
                break;
            }
            cv::Mat frame = cv::Mat(stImageInfo.nHeight,
                                    stImageInfo.nWidth,
                                    CV_8UC3,
                                    pDataForBGR).clone();//官方代码没有用opencv，这里是自己写的

            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();

            int i = *(int*)pUser;
            if(i == 0){
                //输出提示
                ROS_INFO("1111111111111");
//                printf("Cam Serial Number[%s]:GetOneFrame, Width[%d], Height[%d], nFrameNum[%d]\n",
//                       camSerialNumber, stImageInfo.nWidth, stImageInfo.nHeight, stImageInfo.nFrameNum);
                pub1.publish(msg);
            }
            else if(i == 1){
                //输出提示
                ROS_INFO("2222222222");
//                printf("Cam Serial Number[%s]:GetOneFrame, Width[%d], Height[%d], nFrameNum[%d]\n",
//                       camSerialNumber, stImageInfo.nWidth, stImageInfo.nHeight, stImageInfo.nFrameNum);
                pub2.publish(msg);
            }

            free(pDataForBGR);
            pDataForBGR = NULL;
        }
//        ros::spinOnce();  // 处理ROS节点事件循环
    }

    return 0;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "pub_image_dual");
	ros::NodeHandle nh;

	image_transport::ImageTransport it(nh);
	pub1 = it.advertise("HikImage_left", 10);
	pub2 = it.advertise("HikImage_right", 10);


	int nRet = MV_OK;
//	void* handle = NULL;
    void* handle[CAMERA_NUM] = {NULL};
	unsigned char * pData = NULL;
    unsigned char *pDataForBGR = NULL;

//    pthread_t nThreadID[2];
    pthread_t nThreadID[CAMERA_NUM];

	MV_CC_DEVICE_INFO_LIST stDeviceList;
	memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

	nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
	if (MV_OK != nRet)
	{
		printf("MV_CC_EnumDevices fail! nRet [%x]\n", nRet);
		return -1;
	}

	if (stDeviceList.nDeviceNum > 0)
	{
		for (int i = 0; i < stDeviceList.nDeviceNum; i++)
		{
			printf("[device %d]:\n", i);
			MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
			if (NULL == pDeviceInfo)
			{
				break;
			}
			PrintDeviceInfo(pDeviceInfo);
		}
	}
	else
	{
		printf("Find No Devices!\n");
		return -1;
	}

    if(stDeviceList.nDeviceNum < CAMERA_NUM)
    {
        printf("only have %d camera\n", stDeviceList.nDeviceNum);
        return -1;
    }

    // 提示为多相机测试
    // Tips for multicamera testing
    printf("Start %d camera Grabbing Image test\n", CAMERA_NUM);

/// there i go
    for(int i = 0; i < CAMERA_NUM; i++)
    {
        unsigned int nIndex = 0;
        // 选择设备并创建句柄
        // select device and create handle
        printf("Please Input Camera Index: ");
        scanf("%d", &nIndex);

        nRet = MV_CC_CreateHandle(&handle[i], stDeviceList.pDeviceInfo[nIndex]);
        if (MV_OK != nRet)
        {
            printf("MV_CC_CreateHandle fail! nRet [%x]\n", nRet);
            MV_CC_DestroyHandle(handle[i]);
            return -1;
        }
        // 打开设备
        // open device
        nRet = MV_CC_OpenDevice(handle[i]);
        if (MV_OK != nRet)
        {
            printf("MV_CC_OpenDevice fail! nRet [%x]\n", nRet);
            MV_CC_DestroyHandle(handle[i]);
            return -1;
        }

        // ch:探测网络最佳包大小(只对GigE相机有效) | en:Detection network optimal package size(It only works for the GigE camera)
        if (stDeviceList.pDeviceInfo[nIndex]->nTLayerType == MV_GIGE_DEVICE)
        {
            int nPacketSize = MV_CC_GetOptimalPacketSize(handle[i]);
            if (nPacketSize > 0)
            {
                nRet = MV_CC_SetIntValue(handle[i],"GevSCPSPacketSize",nPacketSize);
                if(nRet != MV_OK)
                {
                    printf("Warning: Set Packet Size fail nRet [0x%x]!\n", nRet);
                }
            }
            else
            {
                printf("Warning: Get Packet Size fail nRet [0x%x]!\n", nPacketSize);
            }
        }
    }

    for(int i = 0; i < CAMERA_NUM; i++)
    {
        // @TODO:设置参数，具体看手册《Machine Vision Camera SDK (C)_Developer Guide_V3.2.0_CH》
    	nRet = MV_CC_SetEnumValue(handle[i], "ExposureAuto", 2);
//        nRet = MV_CC_SetEnumValue(handle[i], "ExposureTime", 5000);
        if(MV_OK != nRet){
            printf("MV_CC_SetEnumValue ExposureTime fail! nRet [%x]\n", nRet);
            return -1;
        }
        else
            printf("MV_CC_SetEnumValue ExposureTime success\n");

        // 设置触发模式为off
        // set trigger mode as off
        nRet = MV_CC_SetEnumValue(handle[i], "TriggerMode", 0);
        if (MV_OK != nRet)
        {
            printf("MV_CC_SetTriggerMode fail! nRet [%x]\n", nRet);
            return -1;
        }

        // ch:获取数据包大小 | en:Get payload size
        MVCC_INTVALUE stParam;
        memset(&stParam, 0, sizeof(MVCC_INTVALUE));
        nRet = MV_CC_GetIntValue(handle[i], "PayloadSize", &stParam);
        if (MV_OK != nRet)
        {
            printf("Cam[%d]: Get PayloadSize fail! nRet [0x%x]\n", i, nRet);
            return -1;
        }
        // 开始取流
        // start grab image
        nRet = MV_CC_StartGrabbing(handle[i]);
        if (MV_OK != nRet)
        {
            printf("Cam[%d]: MV_CC_StartGrabbing fail! nRet [%x]\n",i, nRet);
            return -1;
        }
    //////////////////////////////////////////////////////////////////////////////////////
        /*MVCC_STRINGVALUE stStringValue = {0};
        char camSerialNumber[256] = {0};
        nRet = MV_CC_GetStringValue(handle[i], "DeviceSerialNumber", &stStringValue);
        if (MV_OK == nRet)
        {
            memcpy(camSerialNumber, stStringValue.chCurValue, sizeof(stStringValue.chCurValue));
        }
        else
        {
            printf("Get DeviceUserID Failed! nRet = [%x]\n", nRet);
        }

        MV_FRAME_OUT_INFO_EX stImageInfo = {0};
        memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
        pData = (unsigned char *)malloc(sizeof(unsigned char) * stParam.nCurValue);
        if (NULL == pData)
        {
            return -1;
        }
        unsigned int nDataSize = stParam.nCurValue;*/

//////////////////////////////////////////////////////////////////////////
//        pthread_t nThreadID;
        nRet = pthread_create(&nThreadID[i], NULL ,WorkThread , handle[i]);
        if (nRet != 0)
        {
            printf("Cam[%d]: thread create failed.ret = %d\n",i, nRet);
            return -1;
        }
//////////////////////////////////////////////////////////////////////////
//        ros::Rate loop_rate(10);
//        while(ros::ok()){
//            nRet = MV_CC_GetOneFrameTimeout(handle[i], pData, nDataSize, &stImageInfo, 1000);
//            if(MV_OK == nRet)
//            {
//                //printf("Now you GetOneFrame, Width[%d], Height[%d], nFrameNum[%d]\n\n",
//                  //  stImageInfo.nWidth, stImageInfo.nHeight, stImageInfo.nFrameNum);
//                //printf("Image Type: %d", stImageInfo.enPixelType);
//                /*pDataForBGR = (unsigned char *)malloc(MAX_IMAGE_DATA_SIZE);;
//                if (NULL == pDataForBGR)
//                {
//                    return -1;
//                }
//                // 像素格式转换
//                // convert pixel format
//                MV_CC_PIXEL_CONVERT_PARAM stConvertParam = {0};
//                // 从上到下依次是：图像宽，图像高，输入数据缓存，输入数据大小，源像素格式，
//                // 目标像素格式，输出数据缓存，提供的输出缓冲区大小
//                // Top to bottom are：image width, image height, input data buffer, input data size, source pixel format,
//                // destination pixel format, output data buffer, provided output buffer size
//                stConvertParam.nWidth = stImageInfo.nWidth;
//                stConvertParam.nHeight = stImageInfo.nHeight;
//                stConvertParam.pSrcData = pData;
//                stConvertParam.nSrcDataLen = stImageInfo.nFrameLen;
//                stConvertParam.enSrcPixelType = stImageInfo.enPixelType;
//                stConvertParam.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
//                stConvertParam.pDstBuffer = pDataForBGR;
//                stConvertParam.nDstBufferSize = MAX_IMAGE_DATA_SIZE;
//                nRet = MV_CC_ConvertPixelType(handle[i], &stConvertParam);
//                if (MV_OK != nRet)
//                {
//                    printf("MV_CC_ConvertPixelType fail! nRet [%x]\n", nRet);
//                    return -1;
//                }
//                cv::Mat frame = cv::Mat(stImageInfo.nHeight,
//                stImageInfo.nWidth,
//                CV_8UC3,
//                pDataForBGR).clone();//官方代码没有用opencv，这里是自己写的
//
//                sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
//                if(i==0){
//                    pub1.publish(msg);
//                }
//                else if(i==1){
//                    pub2.publish(msg);
//                }*/
//                loop_rate.sleep();
//            }
//        }

    }
//////////////////////////////////////////////////////////////////////////////////////
    for(int i = 0; i < CAMERA_NUM; i++)
    {
        // 停止取流
        // stop grab image
        nRet = MV_CC_StopGrabbing(handle[i]);
        if (MV_OK != nRet)
        {
            printf("MV_CC_StopGrabbing fail! nRet [%x]\n", nRet);
            return -1;
        }
        // 关闭设备
        // close device
        nRet = MV_CC_CloseDevice(handle[i]);
        if (MV_OK != nRet)
        {
            printf("MV_CC_CloseDevice fail! nRet [%x]\n", nRet);
            return -1;
        }
        // 销毁句柄
        // destroy handle
        nRet = MV_CC_DestroyHandle(handle[i]);
        if (MV_OK != nRet)
        {
            printf("MV_CC_DestroyHandle fail! nRet [%x]\n", nRet);
            return -1;
        }
    }
    ros::spin();  // 进入ROS节点的事件循环
	ROS_INFO("TEST");
	return 0;
}

bool PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo)
{
    if (NULL == pstMVDevInfo)
    {
        printf("The Pointer of pstMVDevInfo is NULL!\n");
        return false;
    }
    if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE)
    {
        int nIp1 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
        int nIp2 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
        int nIp3 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
        int nIp4 = (pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);

        // ch:打印当前相机ip和用户自定义名字 | en:print current ip and user defined name
        printf("Device Model Name: %s\n", pstMVDevInfo->SpecialInfo.stGigEInfo.chModelName);
        printf("CurrentIp: %d.%d.%d.%d\n" , nIp1, nIp2, nIp3, nIp4);
        printf("UserDefinedName: %s\n\n" , pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
    }
    else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
    {
        printf("Device Model Name: %s\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chModelName);
        printf("UserDefinedName: %s\n\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
    }
    else
    {
        printf("Not support.\n");
    }

    return true;
}
