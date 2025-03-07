#include "Analyze_Data.h"

//分析数据
void Analyze_Data::setData(uint8_t* c) {
    if(c == 0)
        return;
    int i =0;
    for(;i<Read_dataLen-20 + 1;i++){
        if(int(c[i])==128 && int(c[i+20-1])==127){
            break;
        }
    }
    if(i>=21)
        return;
    color = int(c[i+1]);//判断需要检测的图片类型
    V = int(c[i+2])*1.0/10;//速度
    if(V ==0) V=14;
    SPit = u8Arry2float(&c[i+3]);
    SYaw = u8Arry2float(&c[i+7]);
    y = u8Arry2float(&c[i+11]);
    z = u8Arry2float(&c[i+15]);
//    std::cout<<color<<'\t'<<V<<'\t'<<std::endl;
}


uint8_t * Analyze_Data::TransData() {//数据转换
    uint8_t* data = new uint8_t[Write_dataLen];
    float p = myPitch, y = myYaw;
    float l = position.z;//距离
    data[0]=0x70;//设置数据
    float2u8Arry(&data[1],&y);
    float2u8Arry(&data[5],&p);
    data[9] = int(l/100);
    data[10] = int(l)%100;
    data[Write_dataLen-1]=0x6F;

//    std::cout<<"Fa:\t"<<myPitch<<std::endl;
    return data;
}


void Analyze_Data::setPoint(cv::Point3_<float> point3) {
    position=point3;
}

int Analyze_Data::getColor(){
    return color;
}



void Analyze_Data::analyze() {//对得到的距离进行处理
    double x=position.x;
    double y=position.y;
    double z=position.z;
//    z = int(round(z/100))*100;
    if(z>4500){
        z = z - ((z-4500)*0.22);
    }
    double tan_pitch = getPitch(z*0.001,-y*0.001);//弹道补偿
    double tan_yaw = x / z;
    myYaw = atan(tan_yaw) / CV_PI * 180;
    myPitch = tan_pitch / CV_PI * 180;
//    std::cout<<myPitch<<'\t'<<myYaw<<std::endl;
}




double Analyze_Data::getPitch(double X, double Y) {//弹道补偿
    double theta1 = -3.14 / 4;
    double theta2 = 3.14 / 4;
    double theta_mid;
    double final_theta;
    double y1 = 0;
    double y2 = 0;
    int num = 20;
    for(int i=0; i<num;i++){
        theta_mid = (theta1 + theta2) / 2;
        y1 = (M*G/K) * X/(V* cos(theta1)) +
             tan(theta1)*X +
             M*M*G/(K * K) * log(1 - K*X/(M*V* cos(theta1))) - Y;
        y2 = (M*G/K) * X/(V* cos(theta_mid)) +
             tan(theta_mid) *X +
             M*M*G/(K * K) * log(1 - K*X/(M*V* cos(theta_mid))) - Y;
        double test =  K*X/(M*V* cos(theta1));
        if((y1 * y2) == 0)
        {
            if(y1 == 0)
                final_theta = theta1;
            else
                final_theta = theta2;

            break;
        }
        else if((y1 * y2) < 0)
            theta2 = theta_mid;

        else if(y1 < 0)
            theta1 = theta_mid;
    }
    final_theta = (theta1 + theta2)/2;
//    std::cout<<final_theta<<std::endl;
    return final_theta;
}

cv::Point3_<float> Analyze_Data::getPosition() {
    return position;
}

int Analyze_Data::getNumber() {
    return number;
}

float Analyze_Data::getYaw() {
    return myYaw;
}

float Analyze_Data::getPit() {
    return myPitch;
}

float u8Arry2float(uint8_t *data)
{
    float fa = 0;
    uint8_t uc[4];
    uc[0] = data[0];
    uc[1] = data[1];
    uc[2] = data[2];
    uc[3] = data[3];

    memcpy(&fa, uc, 4);
    return fa;
}

void float2u8Arry(uint8_t *u8Arry, float *floatdata)
{
    uint8_t farray[4];
    *(float *)farray = *floatdata;
    u8Arry[0] = farray[0];
    u8Arry[1] = farray[1];
    u8Arry[2] = farray[2];
    u8Arry[3] = farray[3];

}