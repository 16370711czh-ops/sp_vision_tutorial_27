#ifndef CAMERA_HPP
#define CAMERA_HPP

#include <opencv2/opencv.hpp>
#include "hikrobot/include/MvCameraControl.h"

class Camera
{
public:
    Camera();
    ~Camera();

    void read(cv::Mat& img);

private:
    void* handle_;
};

#endif
