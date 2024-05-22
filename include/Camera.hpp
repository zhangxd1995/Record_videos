//
// Created by zxd on 09.02.23.
//

#ifndef CAM_DISTANCE_CAMERA_HPP
#define CAM_DISTANCE_CAMERA_HPP

#endif //CAM_DISTANCE_CAMERA_HPP

#include <iostream>
#include <vector>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.hpp>
#include <librealsense2/rsutil.h>
#include </home/zxd/CLionProjects/cam_distance/include/Buffer.hpp>
#include <deque>
#include <termios.h>
#include <unistd.h>
#include <fstream>


using namespace Eigen;
using namespace std;
using namespace cv;

class Camera{

private:
    rs2::pipeline pipe;
    rs2::config cfg;
    rs2::pipeline_profile profile;
    rs2::frameset frameset;
    map<string, string> cfg_cam;
    rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams);
    bool profile_changed(const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev);
    float get_depth_scale(rs2::device dev);
public:
    bool hasFrame = false;
    float depth_scale = 0;
    rs2_intrinsics cam_info;
    void setCameraCfg(map<string, string> config);
    void startCamera();
    void startRecord(deque<ImageData> *p);
};

inline void Camera::setCameraCfg(map<string, string> config){
    cfg_cam = config;
    int WIDTH = stoi(cfg_cam["width"]);
    int HEIGHT = stoi(cfg_cam["height"]);
    int fps = stoi(cfg_cam["fps"]);
    cfg.enable_stream(RS2_STREAM_COLOR, WIDTH, HEIGHT, RS2_FORMAT_BGR8, fps);
    cfg.enable_stream(RS2_STREAM_DEPTH, WIDTH, HEIGHT, RS2_FORMAT_Z16, fps);
}

inline void Camera::startCamera(){
    cout << "Starting the Intel RealSense ..." << endl;
    profile = pipe.start(cfg);
    cout << "...started." << endl;
    cam_info = pipe.get_active_profile().get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics();

    //drop out first 60 frames
    for(int i = 0; i < 60; i++)
    {
        //Wait for all configured streams to produce a frame
        frameset = pipe.wait_for_frames(3000);
    }

    ofstream info("/home/zxd/recorded_images/camera_info.txt");
    info << "width: " << cam_info.width << "\n";
    info << "height: " << cam_info.height << "\n";
    info << "fx: " << cam_info.fx << "\n";
    info << "fy: " << cam_info.fy << "\n";
    info << "ppx: " << cam_info.ppx << "\n";
    info << "ppy: " << cam_info.ppy << "\n";
    info <<"coeffs: " << cam_info.coeffs[0] << " " << cam_info.coeffs[1] << " " << cam_info.coeffs[2] << " " << cam_info.coeffs[3] << " " << cam_info.coeffs[4] << "\n";
    info.close();
    hasFrame = true;

}

void Camera::startRecord(deque<ImageData> *p){
    int j = 0;
    rs2::align align_to_color(RS2_STREAM_COLOR);
    rs2_stream align_to = find_stream_to_align(profile.get_streams());
    rs2::align align(align_to);
    auto local = std::chrono::high_resolution_clock::now();
    //start align images
    while(true){
        char c = 0;
        frameset = pipe.wait_for_frames();
        //Get processed aligned frame
        if (profile_changed(pipe.get_active_profile().get_streams(), profile.get_streams())) {
            //If the profile was changed, update the align object, and also get the new device's depth scale
            profile = pipe.get_active_profile();
            align_to = find_stream_to_align(profile.get_streams());
            align = rs2::align(align_to);
            depth_scale = get_depth_scale(profile.get_device());
            cout << "The depth scale is: " << depth_scale << endl;
        }
        auto processed = align.process(frameset);
        //rs2::video_frame other_frame = processed.first(align_to);
        rs2::video_frame color_frame = processed.get_color_frame();
        rs2::depth_frame aligned_depth_frame = processed.get_depth_frame();

        if (!aligned_depth_frame || !color_frame) {
            continue;
        }
        j++;
        auto start = std::chrono::high_resolution_clock::now();
//        if(j>1){
//            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>( start - local).count();
//            cout << "The fps is: "<<1000.0/double(duration) << endl;
//        }
//        local = start;
        int WIDTH = stoi(cfg_cam["width"]);
        int HEIGHT = stoi(cfg_cam["height"]);
        cv::Mat image_color (cv::Size(WIDTH, HEIGHT), CV_8UC3, (void *) color_frame.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat image_depth (cv::Size(WIDTH, HEIGHT), CV_16UC1, (void *) aligned_depth_frame.get_data(), cv::Mat::AUTO_STEP);
        if(j>1) {
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>( start - local).count();
            double capture_fps = 1000.0/double(duration);
            //cv::putText(image_color, "fps: " + to_string(capture_fps), cv::Point(50,50), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                        //cv::Scalar(0, 255, 0), 1);
            cout<< "The current fps is: " << capture_fps <<endl;
        }
        local = start;
        cv::imshow("Image window", image_color);// Show our image inside it.
        // put images to buffer
        buffer_lock.lock();
        p->push_back({image_color.clone(), image_depth.clone()});
        buffer_lock.unlock();
        //cout << "The current buffer size is: " << buffer.size() << endl;
        if (waitKey(1) == 27){
            cout << "-----------------Camera thread ended.--------------------------" << endl;
            cout << "The number of captured images is: " << j << endl;
            break;
        }
    }
}


rs2_stream Camera::find_stream_to_align(const std::vector<rs2::stream_profile>& streams)
{
    //Given a vector of streams, we try to find a depth stream and another stream to align depth with.
    //We prioritize color streams to make the view look better.
    //If color is not available, we take another stream that (other than depth)
    rs2_stream align_to = RS2_STREAM_ANY;
    bool depth_stream_found = false;
    bool color_stream_found = false;

    for (rs2::stream_profile sp : streams)
    {
        rs2_stream profile_stream = sp.stream_type();
        if (profile_stream != RS2_STREAM_DEPTH)
        {
            if (!color_stream_found) //Prefer color
                align_to = profile_stream;

            if (profile_stream == RS2_STREAM_COLOR)
            {
                color_stream_found = true;
            }
        }
        else
        {
            depth_stream_found = true;
        }
    }

    if (!depth_stream_found)
        throw std::runtime_error("No Depth stream available");

    if (align_to == RS2_STREAM_ANY)
        throw std::runtime_error("No stream found to align with Depth");

    return align_to;
}

bool Camera::profile_changed(const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev)
{
    for(auto&& sp : prev)
    {
        //If previous profile is in current (maybe just added another)
        auto itr = std::find_if(std::begin(current), std::end(current), [&sp](const rs2::stream_profile& current_sp) { return sp.unique_id() == current_sp.unique_id(); });
        if (itr == std::end(current)) //If it previous stream wasn't found in current
        {
            return true;
        }
    }
    return false;
}

float Camera::get_depth_scale(rs2::device dev)
{
    // Go over the device's sensors
    for (rs2::sensor& sensor : dev.query_sensors())
    {
        // Check if the sensor if a depth sensor
        if (rs2::depth_sensor dpt = rs2::depth_sensor(sensor))
        {
            return dpt.get_depth_scale();
        }
    }
    throw std::runtime_error("Device does not have a depth sensor");
}




