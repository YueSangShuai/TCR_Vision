#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include "./AutoShoot/TRT/TRTModule.h"

int main() {
    cv::Mat temp=cv::imread("../test/test.jpg");
    cv::VideoCapture capture = cv::VideoCapture("/media/yuesang/G/Robotmaster/dataset/video/video/1.mp4");


    TRTModule model("../AutoShoot/model/model-opt-3.onnx");

    auto start = std::chrono::system_clock::now();
    auto temp1=model(temp);

    auto end = std::chrono::system_clock::now();
    std::cout<<"[INFO]: Average_time "<< std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()<<std::endl;
    std::cout<<temp1.size()<<std::endl;
}
