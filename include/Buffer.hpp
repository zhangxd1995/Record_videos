//
// Created by zxd on 10.02.23.
//

#ifndef CAM_DISTANCE_BUFFER_HPP
#define CAM_DISTANCE_BUFFER_HPP

#endif //CAM_DISTANCE_BUFFER_HPP

#pragma once
#include <iostream>
#include <vector>
#include <mutex>
#include <deque>
#include <opencv2/opencv.hpp>

using namespace std;

struct ImageData
{
    cv::Mat rgb = cv::Mat::zeros(cv::Size(848, 480), CV_8UC3);
    cv::Mat depth = cv::Mat::zeros (cv::Size(848, 480), CV_16UC1);
};

deque<ImageData> buffer;
mutex buffer_lock;




