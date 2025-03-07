#include<iostream>
#include <cstdio>
#include <cstdint>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
using namespace std;

#define Write_dataLen 12//要求的数组长度
#define Read_dataLen 40//要求的数组长度

class SerialPort {
public:
    SerialPort()=default;
    bool OpenSerialPort();//开启串口
    // 重启串口
    void restart_serial();
    // 发送数据
    void send_data(uint8_t a[]);
    // 接收数据
    uint8_t* read_data();
    // 关闭串口
    void close_port();

private:
    const char* file_name_="/dev/ttyUSB0";//设置串口名字
    int fd=-1;//fd为打开的终端文件描述符
};
