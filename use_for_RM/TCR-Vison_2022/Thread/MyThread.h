#pragma once

#include "../HaikangCamera/CameraRGB/HaiKangCameraRGB.h"
#include "../SerialPort/SerialPort.h"
#include "../AutoShoot/TRT/TRTModule.h"
#include "../Analyze_data/Analyze_Data.h"
#include "../PNP_Distance/PNP_Distance.h"
#include <thread>
#include <mutex>
#include <functional>
#include <condition_variable>
#include "yaml-cpp/yaml.h"
#include <fstream>
#include<chrono>
#include "../Kalman/Kalman.h"

using namespace std::chrono;

extern int debug_level;//bug等级

void RePicture(const int &);

void ReVideo(const int &);

void ReCamera(const int &);

string getTime(tm* t);//解析时间返回字符串
class
MyThread {
private:
    SerialPort serialPort;//串口通信
    Analyze_Data analyzeData;//数据分析
    PNP_Distance pnpDistance;//测距模块

    HaiKangCameraRGB HaiKang;//相机
    int color = 0; //需要击打的颜色

    int last_id = -1;//上一次击打的目标

    bool isContinue = true;//判断是否继续

    bool SaveVideo_Ok = false;

    VideoWriter wri;

    time_t timeReal;
    int encode_type = VideoWriter::fourcc('M', 'J', 'P', 'G');   //选择编码格式


    Mat frame;       //图像
    mutex mutexLock;                                    // 互斥锁
    condition_variable CarmeraCondition,DetectCondition;  // 条件变量 相机
public:
    MyThread(){
        std::cout<<"wait openSerialPort"<<std::endl;
        while(!serialPort.OpenSerialPort());//开启串口
        std::cout<<"wait openCamera"<<std::endl;
        while(HaiKang.StartDevice(0) == -1){
//        std::cout<<"wait camera"<<std::endl;
            ;
        }
        std::cout<<"over"<<std::endl;
        pnpDistance.setData();  //初始化
    }

    Kalman kalman;
    void VInthread();//视频读线程

    void CInthread();//相机读线程
    void decThread();//识别线程
    void Prethread();//绘图线程

    void SendPort();//发送数据给串口
    void ReceivePort();//接受串口来的数据

    void setImg(const Mat&);//设置图片
    void setEmpty();//设置为空
    void setColor(const int &);
    void writeFrame(const int&);


    bool Decision(vector<bbox_t>,bbox_t&);//对获取的数字装甲板进行优先级排序
};

