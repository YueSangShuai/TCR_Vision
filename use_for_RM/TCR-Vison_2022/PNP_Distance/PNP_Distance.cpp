//
// Created by rmtcr on 2022/10/25.
//

#include "PNP_Distance.h"
void PNP_Distance::setData() {
    array<double, 9> matrix;
    matrix[0]=4036.732876233236 ;
    matrix[1]=0;
    matrix[2]=319.4481235307201 + 15;//+ left - right
    matrix[3]=0;
    matrix[4]=3988.295682670543;
    matrix[5]=191.6284608921223 -40;// + up - down
    matrix[6]=0;
    matrix[7]=0;
    matrix[8]=1.0000;
    array<double, 5>  coeffs;
    coeffs[0]=0.3973560281128358;
    coeffs[1]=-40.99838489999142;
    coeffs[2]=-0.02770590683932514;
    coeffs[3]=-0.02799904870014409;
    coeffs[4]=0;
    //获取相机参数
    //相机内参矩阵
    cameraMatrix = Mat(3, 3, CV_64F, const_cast<double *>(matrix.data())).clone();
    //相机畸变参数
    distCoeffs = Mat(1, 5, CV_64F, const_cast<double *>(coeffs.data())).clone();

    float s_h_x = kSmallArmorWidth / 2.0f;//small half x
    float s_h_y = kSmallArmorHeight / 2.0f;//small half y
    float l_h_x = kLargeArmorWidth / 2.0f;
    float l_h_y = kLargeArmorHeight / 2.0f;

    //设置大小装甲板点位
    small_points.emplace_back(Point3f(-s_h_x, s_h_y, 0));
    small_points.emplace_back(Point3f(-s_h_x, -s_h_y, 0));
    small_points.emplace_back(Point3f(s_h_x, -s_h_y, 0));
    small_points.emplace_back(Point3f(s_h_x, s_h_y, 0));

    large_points.emplace_back(Point3f(-l_h_x, l_h_y, 0));
    large_points.emplace_back(Point3f(-l_h_x, -l_h_y, 0));
    large_points.emplace_back(Point3f(l_h_x, -l_h_y, 0));
    large_points.emplace_back(Point3f(l_h_x, l_h_y, 0));
}


//低通滤波
double PNP_Distance::Di_filter(double new_data)
{
    if(old_data == 0){
        old_data = new_data;
        return old_data;
    }
    if (new_data - old_data > 0 )       //计算方向
        new_flag = 1;
    else new_flag = 0;
    if (new_flag == old_flag)           //变化同向
    {
        if (abs (new_data - old_data) > Threshold_1)    //变化很大则num增大
            num_x += 5;
        if (num_x >= Threshold_2)       //变化持续同向且一直变化很大则k_x增大
            k_x += 0.2;
    }
    else                //变化反向
    {
        num_x = 0;
        k_x = 0.2;
        old_flag = new_flag;
    }
    if (k_x > 0.95)  k_x = 0.95;    //系数限幅
    new_data = (1-k_x) * old_data + k_x * new_data;   //计算答案

    old_data = new_data;        //更新old_data

    return new_data;
}

bool PNP_Distance::TosolvePnP(const bbox_t &armor, Point3f &point) {
    vector<Point2f> armorP;
    //获取装甲板各点位置
    for(int i=0;i<4;i++){
        armorP.emplace_back(armor.pts[i]);
    }

    // Solve pnp

    vector<Point3f> object_points = armor.type == SMALL
                                    ? small_points : large_points;

    bool success = solvePnP(
            object_points, armorP,
            cameraMatrix, distCoeffs, //相机内参矩阵 相机畸变参数
            rvec, tvec, false,
            SOLVEPNP_IPPE);
    if (success) {//获取坐标位置
        point.x = tvec.at<double>(0);
        point.y = tvec.at<double>(1);
        point.z = Di_filter(tvec.at<double>(2)); //低通滤波


        return true;
    }
    return false;
}


