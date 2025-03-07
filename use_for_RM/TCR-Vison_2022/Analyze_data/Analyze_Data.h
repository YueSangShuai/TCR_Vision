#include <cstdio>
#include <cstdint>
#include<iostream>
#include <fcntl.h>
#include <termios.h>
#include <complex>
#include <unistd.h>
#include <opencv2/core/types.hpp>
#include "../Struct.h"

#define Write_dataLen 12//要求的数组长度
#define Read_dataLen 40//要求的数组长度

using namespace std;

void float2u8Arry(uint8_t *u8Arry, float *floatdata);
float u8Arry2float(uint8_t *data);

class Analyze_Data{//用于分析串口传入的数据
private:
    uint8_t head=0x80,end=0x7F;//用于了解哪里开始是头 哪些结束是尾
    cv::Point3_<float> position;
    int color=-1;//颜色
    int number=-1;//目标序号

    double V=14;//子弹速度
    double K=0.0046;//风阻 0.0046
    double M = 0.003;//质量 43da 03xiao
    double G = 9.78;//重力加速度

public:
    //四元数
    float w = INT_MAX;
    float x = INT_MAX;
    float y = INT_MAX;
    float z = INT_MAX;

    //获取的pit yaw
    float SPit = 0;
    float SYaw = 0;
private:
    float myPitch=0;
    float myYaw=0;
public:
    bool isSend=false;//判断是否在发送

    Analyze_Data()=default;

    void setData(uint8_t* c);

    unsigned char * TransData();
    int getNumber();

    void setPoint(cv::Point3_<float> point3);

    cv::Point3_<float> getPosition();
    int getColor();
    void analyze();
    float getPit();
    float getYaw();

    double getPitch(double x,double y);



};