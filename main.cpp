#include <iostream>
#include <fstream>
#include <string>
#include <map>
#include <vector>
#include <thread>
#include <mutex>
//#include <boost/filesystem.hpp>
#include </home/zxd/CLionProjects/cam_distance/include/Camera.hpp>
#include </home/zxd/CLionProjects/cam_distance/include/StoreImage.hpp>
#include </home/zxd/CLionProjects/cam_distance/include/Buffer.hpp>

using namespace std;

string trim(string s){
    if (!s.empty()){
        s.erase(0,s.find_first_not_of(" "));
        s.erase(s.find_last_not_of(" ") + 1);
    }
    return s;
}

void thread_cam(map<string, string> config, Camera cam) {

    cam.startRecord(&buffer);
}

void thread_store(map<string, string> config) {
    StoreImage store;
    store.setStoreCfg(config);
    store.startStoring(&buffer);
}


int main(int argc, char** argv) {

    //// read config ////
    map<string, string> config;
    // Open the configuration file
    string config_path = "/home/zxd/CLionProjects/cam_distance/config.txt";
    ifstream file(config_path);
    if (!file.is_open()) {
        cerr << "Error: Unable to open configuration file." << std::endl;
        return 1;
    }
    // Read the contents of the file into a map
    string line;
    while (getline(file, line)) {
        size_t pos = line.find('=');
        if (pos != string::npos) {
            string key = trim(line.substr(0, pos));
            string value = trim(line.substr(pos + 1));
            config[key] = value;
        }
    }
    // Close the file
    file.close();

    //// Start camera thread and store thread  ////
    Camera cam;
    cam.setCameraCfg(config);
    cam.startCamera();
    cv::namedWindow("Image window", cv::WINDOW_AUTOSIZE); // Create a window for display.
    if (cam.hasFrame == true){
        thread cam_thread(thread_cam,config,cam);
        thread store_thread(thread_store,config);
        cam_thread.join();
        store_thread.join();
    }


//    Camera cam;
//    cam.setCameraCfg(config);
//    cam.startCamera();
//    cam.startRecord(&buffer);
    //cout << "after camera buffer size: " << buffer.size() << endl;

//    auto first_pair = buffer.front();
//
//    // Access the two Mat objects in the pair
//    cv::Mat color = first_pair.rgb;
//    cv::Mat depth = first_pair.depth;
//    cout << buffer.empty() << endl;
//    //cv::namedWindow("Color Image", cv::WINDOW_AUTOSIZE);
//    cv::imshow("Image", color);
//    waitKey();
//
//    StoreImage store;
//    store.setStoreCfg(config);
//    store.startStoring(&buffer);

    //system ("pause");
    return 0;
}
