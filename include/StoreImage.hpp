//
// Created by zxd on 09.02.23.
//

#ifndef CAM_DISTANCE_STOREIMAGE_HPP
#define CAM_DISTANCE_STOREIMAGE_HPP

#endif //CAM_DISTANCE_STOREIMAGE_HPP

#include <iostream>
#include <boost/filesystem.hpp>
#include <sstream>
#include <deque>
#include <vector>

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.hpp>
#include <librealsense2/rsutil.h>
#include </home/zxd/CLionProjects/cam_distance/include/Buffer.hpp>

using namespace Eigen;
using namespace std;
using namespace cv;

class StoreImage{

private:
    map<string, string> cfg_store;
    string dir_path_rgb;
    string dir_path_depth;
    bool isAllZeros(const cv::Mat& mat);
public:
    void setStoreCfg(map<string, string> config);
    void startStoring(deque<ImageData> *p);
};

void StoreImage:: setStoreCfg(map<string, string> config) {
    cfg_store = config;
    //// make dir for saving images ////
    // The path of the directory you want to check
    string save_dir = cfg_store["save_dir"];;
    string actor = cfg_store["actor"];
    string task = cfg_store["task"];
    string date = cfg_store["date"];
    dir_path_rgb = save_dir + actor;
    dir_path_depth = save_dir + actor;

    if (!boost::filesystem::exists(boost::filesystem::path(dir_path_rgb))) {
        boost::filesystem::create_directory(boost::filesystem::path(dir_path_rgb));
    }
    dir_path_rgb = dir_path_rgb + task;
    if (!boost::filesystem::exists(boost::filesystem::path(dir_path_rgb))) {
        boost::filesystem::create_directory(boost::filesystem::path(dir_path_rgb));
    }
    dir_path_rgb = dir_path_rgb + "RGB_images/";
    if (!boost::filesystem::exists(boost::filesystem::path(dir_path_rgb))) {
        if (boost::filesystem::create_directory(boost::filesystem::path(dir_path_rgb))) {
            std::cout << "RGB directory created successfully." << std::endl;
        }
    }

    if (!boost::filesystem::exists(boost::filesystem::path(dir_path_depth))) {
        boost::filesystem::create_directory(boost::filesystem::path(dir_path_depth));
    }
    dir_path_depth = dir_path_depth + task;
    if (!boost::filesystem::exists(boost::filesystem::path(dir_path_depth))) {
        boost::filesystem::create_directory(boost::filesystem::path(dir_path_depth));
    }
    dir_path_depth = dir_path_depth + "depth_images/";
    if (!boost::filesystem::exists(boost::filesystem::path(dir_path_depth))) {
        if (boost::filesystem::create_directory(boost::filesystem::path(dir_path_depth))) {
            std::cout << "Depth directory created successfully." << std::endl;
        }
    }
}

void StoreImage::startStoring(deque<ImageData> *p){
    int i = 0;
    string type_rgb = cfg_store["type_rgb"];
    string type_depth = cfg_store["type_depth"];
    cout << "Starting storing" << endl;
    // local1 = std::chrono::high_resolution_clock::now();
    while(true) {
        if (!p->empty()) {
            //auto start1 = std::chrono::high_resolution_clock::now();
            buffer_lock.lock();
            auto first_pair = p->front();
            buffer_lock.unlock();
            if (!first_pair.rgb.empty() && !first_pair.depth.empty()) {
                //save images to local
                ostringstream oss_rgb;
                oss_rgb << dir_path_rgb << i << type_rgb;
                string filename_rgb = oss_rgb.str();
                //cout << filename_rgb << endl;

                ostringstream oss_depth;
                oss_depth << dir_path_depth << i << type_rgb;
                string filename_depth = oss_depth.str();
                //cout << filename_depth << endl;

                cv::imwrite(filename_rgb, first_pair.rgb);
                cv::imwrite(filename_depth, first_pair.depth);
                buffer.pop_front();
                i++;
//                if(i>1) {
//                    auto duration1 = std::chrono::duration_cast<std::chrono::milliseconds>(start1 - local1).count();
//                    cout << "The store fps is: " << 1000.0 / double(duration1) << endl;
//                }
//                local1 = start1;
//                auto end1 = std::chrono::high_resolution_clock::now();
//                auto duration2 = std::chrono::duration_cast<std::chrono::milliseconds>(start1 - end1).count();
//                cout << "The store time is: " << double(duration2) << endl;
            } else {
                buffer.pop_front();
            }
        }
        else{
            waitKey(1000);
            if (p->empty()){
                cout << "-----------------Store thread ended.--------------------------" << endl;
                cout << "The number of stored images is: " << i << endl;
                break;
            }
            else
                continue;
        }
    }
}

bool StoreImage::isAllZeros(const cv::Mat& mat) {
    cv::MatConstIterator_<uchar> it = mat.begin<uchar>(), it_end = mat.end<uchar>();
    for (; it != it_end; ++it) {
        if (*it != 0) {
            return false;
        }
    }
    return true;
}