#include "camera.h"
#include "MvCameraControl.h"
#include "CameraParams.h"
#include <cstring>
#include <cstdio>
#include <unistd.h>
#include "HIK_camera.h"

#define MAX_IMAGE_DATA_SIZE (4 * 2048 * 2448)

namespace ly{
    HIK_camera::HIK_camera(int deviceIndex)
    {
        int nRet = MV_OK;
        MV_CC_DEVICE_INFO_LIST stDeviceList;
        memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

        // 枚举设备
        // enum device
        nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
        // 枚举失败或没有找到设备
        if (MV_OK != nRet || stDeviceList.nDeviceNum <= 0)
        {
            printf("MV_CC_EnumDevices fail! nRet [%x]\n", nRet);
            return ;
        }

        if (deviceIndex < 0 || deviceIndex >= static_cast<int>(stDeviceList.nDeviceNum)) {
            printf("Invalid device index: %d\n", deviceIndex);
            return;
        }

        // 选择设备并创建句柄
        // select device and create handle
        nRet = MV_CC_CreateHandle(&handle_, stDeviceList.pDeviceInfo[deviceIndex]);
        if (MV_OK != nRet)
        {
            printf("MV_CC_CreateHandle fail! nRet [%x]\n", nRet);
            return;
        }

        // 打开设备
        // open device
        nRet = MV_CC_OpenDevice(handle_);
        if (MV_OK != nRet)
        {
            printf("MV_CC_OpenDevice fail! nRet [%x]\n", nRet);
            MV_CC_DestroyHandle(handle_);
            return;
        }

        nRet = MV_CC_SetEnumValue(handle_, "ExposureAuto", 0);//要专门关闭自动曝光，不然后面设置手动曝光参数可能报错
        if(MV_OK != nRet){
            printf("MV_CC_SetEnumValue ExposureAuto fail! nRet [%x]\n", nRet);
            return;
        }
        else
            printf("MV_CC_SetEnumValue ExposureAuto success\n");
        nRet = MV_CC_SetFloatValue(handle_, "ExposureTime", 10000);
        if(MV_OK != nRet){
            printf("MV_CC_SetEnumValue ExposureTime fail! nRet [%x]\n", nRet);
            return;
        }
        else
            printf("MV_CC_SetEnumValue ExposureTime success\n");

        nRet = MV_CC_SetFloatValue(handle_, "Gain", 15.0);
        if(MV_OK != nRet){
            printf("MV_CC_SetEnumValue Gain fail! nRet [%x]\n", nRet);
            return;
        }
        else
            printf("MV_CC_SetEnumValue Gain success\n");

        nRet = MV_CC_SetBoolValue(handle_, "AcquisitionFrameRateEnable", false);
        if (MV_OK != nRet)
        {
            printf("set AcquisitionFrameRateEnable fail! nRet [%x]\n", nRet);
            return;
        }

        // 设置触发模式为off
        // set trigger mode as off
        nRet = MV_CC_SetEnumValue(handle_, "TriggerMode", 1);
        if (MV_OK != nRet)
        {
            printf("MV_CC_SetTriggerMode fail! nRet [%x]\n", nRet);
            return;
        }

        // 设置触发源
        // set trigger source
        nRet = MV_CC_SetEnumValue(handle_, "TriggerSource", MV_TRIGGER_SOURCE_LINE2);
        if (MV_OK != nRet)
        {
            printf("MV_CC_SetTriggerSource fail! nRet [%x]\n", nRet);
            return;
        }

        // 开始取流
        // start grab image
        nRet = MV_CC_StartGrabbing(handle_);
        if (MV_OK != nRet)
        {
            printf("MV_CC_StartGrabbing fail! nRet [%x]\n", nRet);
            return;
        }
        // 分配存储图片的空间
        // ch:获取数据包大小 | en:Get payload size
        MVCC_INTVALUE stParam;
        memset(&stParam, 0, sizeof(MVCC_INTVALUE));
        nRet = MV_CC_GetIntValue(handle_, "PayloadSize", &stParam);
        if (MV_OK != nRet)
        {
            printf("Get PayloadSize fail! nRet [0x%x]\n", nRet);
            return ;
        }
        memset(&stImageInfo_, 0, sizeof(MV_FRAME_OUT_INFO_EX));
        pData_ = (unsigned char *)malloc(sizeof(unsigned char) * stParam.nCurValue);
        nDataSize_ = stParam.nCurValue;

    }
    HIK_camera::~HIK_camera()
    {
        free(pDataForRGB_);
    }

    cv::Mat HIK_camera::getFrame()
    {
        cv::Mat val_out;
        int nRet;
        nRet = MV_CC_GetOneFrameTimeout(handle_, pData_, nDataSize_, &stImageInfo_, 1000);
        if (nRet == MV_OK)
        {
            if(first_)
            {
//                pDataForRGB_ = (unsigned char*)malloc(stImageInfo_.nWidth * stImageInfo_.nHeight * 4 + 2048);
                pDataForRGB_ = (unsigned char*)malloc(MAX_IMAGE_DATA_SIZE);
                // 图片格式转换参数
                stConvertParam_.nWidth = stImageInfo_.nWidth;
                stConvertParam_.nHeight = stImageInfo_.nHeight;
                stConvertParam_.pSrcData = pData_;
                stConvertParam_.nSrcDataLen = stImageInfo_.nFrameLen;
                stConvertParam_.enSrcPixelType = stImageInfo_.enPixelType;
                stConvertParam_.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
                stConvertParam_.pDstBuffer = pDataForRGB_;
//                stConvertParam_.nDstBufferSize = stImageInfo_.nWidth * stImageInfo_.nHeight *  4 + 2048;
                stConvertParam_.nDstBufferSize = MAX_IMAGE_DATA_SIZE;
                first_ = false;
            }
            nRet = MV_CC_ConvertPixelType(handle_, &stConvertParam_);
            if (MV_OK != nRet)
            {
                printf("MV_CC_ConvertPixelType fail! nRet [%x]\n", nRet);
                return cv::Mat();
            }
            auto img = cv::Mat(stImageInfo_.nHeight, stImageInfo_.nWidth, CV_8UC3, pDataForRGB_);
            return img;
        }
        else{
            printf("No data[%x]\n", nRet);
        }
    }
}

