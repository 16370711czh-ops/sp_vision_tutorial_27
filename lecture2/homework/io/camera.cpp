#include "camera.hpp"

#include <unordered_map>


cv::Mat transfer(MV_FRAME_OUT& raw)
{
    cv::Mat raw_img(
        cv::Size(raw.stFrameInfo.nWidth,
                 raw.stFrameInfo.nHeight),
        CV_8U,
        raw.pBufAddr
    );

    auto pixel_type = raw.stFrameInfo.enPixelType;

    const static std::unordered_map<
        MvGvspPixelType,
        cv::ColorConversionCodes
    > type_map = {
        {PixelType_Gvsp_BayerGR8, cv::COLOR_BayerGR2RGB},
        {PixelType_Gvsp_BayerRG8, cv::COLOR_BayerRG2RGB},
        {PixelType_Gvsp_BayerGB8, cv::COLOR_BayerGB2RGB},
        {PixelType_Gvsp_BayerBG8, cv::COLOR_BayerBG2RGB}
    };

    cv::Mat img;

    cv::cvtColor(
        raw_img,
        img,
        type_map.at(pixel_type)
    );

    return img;
}


Camera::Camera()
{
    handle_ = nullptr;

    int ret;

    MV_CC_DEVICE_INFO_LIST device_list{};

    ret = MV_CC_EnumDevices(
        MV_USB_DEVICE,
        &device_list
    );

    if (ret != MV_OK)
        return;

    if (device_list.nDeviceNum == 0)
        return;

    ret = MV_CC_CreateHandle(
        &handle_,
        device_list.pDeviceInfo[0]
    );

    if (ret != MV_OK)
        return;

    ret = MV_CC_OpenDevice(handle_);

    if (ret != MV_OK)
    {
        MV_CC_DestroyHandle(handle_);
        handle_ = nullptr;
        return;
    }

    MV_CC_SetEnumValue(
        handle_,
        "BalanceWhiteAuto",
        MV_BALANCEWHITE_AUTO_CONTINUOUS
    );

    MV_CC_SetEnumValue(
        handle_,
        "ExposureAuto",
        MV_EXPOSURE_AUTO_MODE_OFF
    );

    MV_CC_SetEnumValue(
        handle_,
        "GainAuto",
        MV_GAIN_MODE_OFF
    );

    MV_CC_SetFloatValue(
        handle_,
        "ExposureTime",
        10000
    );

    MV_CC_SetFloatValue(
        handle_,
        "Gain",
        16.9
    );

    MV_CC_SetFrameRate(
        handle_,
        60
    );

    ret = MV_CC_StartGrabbing(handle_);

    if (ret != MV_OK)
    {
        MV_CC_CloseDevice(handle_);
        MV_CC_DestroyHandle(handle_);
        handle_ = nullptr;
        return;
    }
}


void Camera::read(cv::Mat& img)
{
    if (handle_ == nullptr)
        return;

    MV_FRAME_OUT raw{};

    unsigned int nMsec = 100;

    int ret = MV_CC_GetImageBuffer(
        handle_,
        &raw,
        nMsec
    );

    if (ret != MV_OK)
        return;

    img = transfer(raw);

    MV_CC_FreeImageBuffer(
        handle_,
        &raw
    );
}


Camera::~Camera()
{
    if (handle_ == nullptr)
        return;

    MV_CC_StopGrabbing(handle_);

    MV_CC_CloseDevice(handle_);

    MV_CC_DestroyHandle(handle_);
}
