#ifndef MY_UVC_H
#define MY_UVC_H

#include "zf_common_headfile.h" 
#include <iostream>
#include <vector>
using namespace std;
#include <opencv2/opencv.hpp>
using namespace cv;
// int8
#include <zf_common_typedef.h>

int8 my_uvc_camera_init(const char *path);
//此处参数需要根据摄像头更改
#define UVC_WIDTHRAW  160   
#define UVC_HEIGHTRAW 120   
#define UVC_FPSRAW    120 

int8 wait_image_refresh();

void calculateFPS(void);

int countCenterWhitePixels(const cv::Mat &img);

cv::Mat smooth_edges_gaussian(cv::Mat binary, int kernel_size, double sigma);

#endif