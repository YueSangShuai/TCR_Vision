#include <iostream>
#include <fstream>
#include "./HaikangCamera/CameraGray/HaiKangCameraGray.h"
#include"./HaikangCamera/CameraRGB/HaiKangCameraRGB.h"
#include "yaml-cpp/yaml.h"

int main() {
    HaiKangCameraGray HaiKang;
//    HaiKangCameraRGB HaiKang;

    auto config=YAML::LoadFile("/home/yuesang/Project/CLionProjects/Haikang/yaml/camera.yaml");

    HaiKang.StartDevice(0);
    // 设置分辨率
    HaiKang.SetResolution(config["ResolutionW"].as<int>(), config["ResolutionH"].as<int>());
    // 开始采集帧
    HaiKang.SetStreamOn();
    // 设置曝光事件
    HaiKang.SetExposureTime(config["ExposureTime"].as<int>());
    //增益设置
//    if(config["setGain"].as<bool>()){
//         HaiKang.SetGAIN(0, 16);
//         HaiKang.SetGAIN(1, 8);
//         HaiKang.SetGAIN(2, 8);
//         HaiKang.SetGAIN(3, 16);
//    }



    // 是否启用自动白平衡7
    if(config["setAutoGain"].as<bool>()){
        HaiKang.Set_Auto_BALANCE();
    }

    // manual白平衡 BGR->012
    if(config["Set_BALANCE"].as<bool>()){
        HaiKang.Set_BALANCE(0, config["B_Gain"].as<int>());
        HaiKang.Set_BALANCE(1, config["G_Gain"].as<int>());
        HaiKang.Set_BALANCE(2, config["R_Gain"].as<int>());
    }


    while(true){
        cv::Mat frame;
        auto HaiKang_stauts = HaiKang.GetMat(frame);
        cv::imshow("aaa",frame);
        cv::waitKey(1);
    }
    return 0;
}
