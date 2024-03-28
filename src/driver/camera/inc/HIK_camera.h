#ifndef __HIK_CAMERA_H
#define __HIK_CAMERA_H

#include "camera.h"
#include "CameraParams.h"

namespace ly{
    class HIK_camera:public camera
    {
    public:
//        HIK_camera();
        HIK_camera(int deviceIndex = 0); // 默认值为0，允许向后兼容
        ~HIK_camera();

        cv::Mat getFrame() override;
        int getType() override{ return HIK_CAM;}
    private:
        void* handle_ = NULL;
        MV_FRAME_OUT_INFO_EX stImageInfo_ = {0};
        unsigned char * pData_;
        unsigned int nDataSize_;
        unsigned char *pDataForRGB_ = NULL;
        MV_CC_PIXEL_CONVERT_PARAM stConvertParam_ = {0};
        bool first_ = true;
    };
}

#endif //__HIK_CAMERA_H
