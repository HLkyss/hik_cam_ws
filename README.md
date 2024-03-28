# hik_cam_ws
海康相机采集图像ros节点<br />
包含普通模式和外部触发模式采集

编译：<br />
cd /home/vio/Code/hik_vio_ws/src/HikRobot-Camera-ROS-main/src/driver/camera <br />
mkdir build && cd build <br />
cmake .. <br />
make <br />
<br />
cd /home/vio/Code/hik_vio_ws <br />
catkin_make <br />


<img src="https://github.com/HLkyss/hik_cam_ws/assets/69629475/74271f8e-a11e-41b3-8a2d-47b9e59cc652" width="800"> <br />
<img src="https://github.com/HLkyss/hik_cam_ws/assets/69629475/d697a3c0-6625-461c-9fc0-83759731b98c" width="800"> <br />
<img src="https://github.com/HLkyss/hik_cam_ws/assets/69629475/5f6d9867-6989-44ee-9f28-5548dde11943" width="800"> <br />

同步触发板及程序链接：[嘉立创](https://oshwhub.com/hlkyss/naze32-fei-kong_copy_copy_copy_copy) </br>
<img src="https://github.com/HLkyss/hik_cam_ws/assets/69629475/93b6f06d-d0b2-48b0-94ea-e6397a290e14" width="400"> <br />
<img src="https://github.com/HLkyss/hik_cam_ws/assets/69629475/766b62fc-2a1e-4f9d-943e-4582d2dbfb9d" width="400"> <br />

采集平台搭建：<br />
<img src="https://github.com/HLkyss/hik_cam_ws/assets/69629475/434f4165-0ada-4fcc-9a75-b276af121632" width="800"> <br />
<img src="https://github.com/HLkyss/hik_cam_ws/assets/69629475/f63d46a1-44de-412b-b02d-58fa556800f7" width="800"> <br />
<img src="https://github.com/HLkyss/hik_cam_ws/assets/69629475/ba8c4cd9-af5a-4911-9260-dce005709d00" width="800"> <br />


***
经过比对网上开源的ros版本的海康相机代码，大致了解了工作流程：

1. 根据官方sdk代码修改：/opt/MVS/Samples/64，此路径包含所有官方代码。并且包含一个readme说明了每个示例代码的主要功能。
主要用到：
 - GrabImage：主动方式抓取图像
 - ImageProcess：图像处理(存图和像素格式转换)

对比了pub_image_right.cpp和GrabImage.cpp，并将对比过程中的细节记录在了代码里。

2. 根据上述经验，理解了开源代码怎样将官方代码应用到ros中。于是自己写针对多相机的代码pub_image_dual.cpp，主要参考：
 - GrabMultipleCamera：多相机取流

运行pub_image_dual2.cpp即可。

3. 补充触发模式节点：
运行pub_image_dual_trigger.cpp即可

官方提供了更多细节信息，包括完整的安装过程、各种注意事项、代码接口细节等，值得仔细去看：
/opt/MVS/doc/Machine Vision Camera SDK (C)_Developer Guide_V3.2.0_CH

其中“环境配置”中就包含了一个重要补充信息：“使用USB相机建议设置usb缓存（set_usbfs_memory_size.sh）。set_usbfs_memory_size.sh：设置usb缓存，使之能兼容多个大分辨率u3相机。”
另外里面还有“相机参数节点表”，列出了所有相机参数细节。

遇到的问题：最开始怎么改代码都会遇到相机画面卡顿或撕裂的问题，用官方软件也有类似问题，困扰了很久。后面证实是由于USB带宽不稳定造成的，使用的相机为USB3.0相机，在USB3.2接口的电脑上会出现过低或不均的情况：<br />
<img src="https://github.com/HLkyss/hik_cam_ws/assets/69629475/951e0632-4525-41e7-81c0-e1b914139026" width="200"> <br />
<img src="https://github.com/HLkyss/hik_cam_ws/assets/69629475/eb4618bf-b3e3-4927-b72c-2648b80a0a3a" width="800"> <br />
<img src="https://github.com/HLkyss/hik_cam_ws/assets/69629475/4d60f0bd-9516-4345-95ce-aef428d08b7a" width="800"> <br />

4. 内存泄漏问题 <br />
解决了占用内存过大的问题，并移植到工控机上。<br />
之前应该是，每次都会重新申请指针指向新的地址存数据，这一版改用类，每个相机类只申请一次内存，以后都在同一个地址存新数据。尝试free掉，失败。<br />
新程序解决内存泄漏问题pub_image_dual_trigger2.cpp <br />
<br />
问题： <br />
工控机上有个问题：有一个文件/home/hl/project/camera/hik_cam_ros/hik_4_ws/src/HikRobot-Camera-ROS-main/src/driver/camera/inc/HIK_camera.h <br />
先编译camera库时，保持MV_CC_PIXEL_CONVERT_PARAM stConvertParam_ = {0};不变。再编译整个ros工作空间时会报错，要修改为：MV_CC_PIXEL_CONVERT_PARAM_EX stConvertParam_ = {0}; <br />
经过多次测试，这个问题在我的机子上没出现过，可能是安装的mvs相关库的版本不一样。 <br />

