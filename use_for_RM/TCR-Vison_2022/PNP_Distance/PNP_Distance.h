//
// Created by rmtcr on 2022/10/25.
//
#pragma once
#include "../Struct.h"
#include <Eigen/Core>
#include<opencv2/opencv.hpp>
#include<opencv2/core/eigen.hpp>

#include<vector>
#include<iostream>

#include "sophus/se3.hpp"
#include "sophus/so3.hpp"
using namespace std;
using namespace cv;
using namespace Sophus;


class PNP_Distance {
private:
    Mat cameraMatrix;//相机的内参矩阵
    Mat distCoeffs;//相机的畸变系数

    //大小装甲板高度与宽度
    const float kSmallArmorWidth = 135;
    const float kSmallArmorHeight = 55;
    const float kLargeArmorWidth = 225;
    const float kLargeArmorHeight = 55;

    //大小装甲板位置xyz
    vector<Point3f> small_points;
    vector<Point3f> large_points;


    int old_flag = 0;   //表示第n-2个数据到第n-1个数据的变化趋势，加为1，减为0
    int new_flag = 0;   //表示第n-1个数据到第n个数据的变化趋势，加为1，减为0
    double old_data = 0;//上一帧的距离
    int num_x = 0;      //一个过渡值，对于同方向变化量大的数据，num_x越大
    float k_x = 0.2;    //表示对新读入的数据的信任度，取值范围0-1

    const double Threshold_1 = 100;
    const double Threshold_2 = 8;
    Mat rvec, tvec; //rvec表示世界坐标系中的地i个坐标周方向的单位向量在
public:
    PNP_Distance() = default;
    void setData();//设置参数
    PNP_Distance(const array<double, 9> &matrix, const vector<double> &coeffs) {
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

    // Get 3d position
    bool TosolvePnP(const bbox_t &, Point3f &);

    double Di_filter(double);

public:

    Sophus::SE3<double> armor2Camera;
    Sophus::SE3<double> camera2gimbal;

    Sophus::SE3<double> armor2world;
    Sophus::SE3<double> gimbal2world;
    Sophus::SE3<double> camera2world;
    Mat world;
    Eigen::MatrixXd world_Eigen;
    void setimu(double pitch,double yaw){

    }
    void getrescult(double pitch,double yaw){
        double yaw_theta=yaw/57.2958f;
        double pitch_theta=pitch/57.2958f;

        double pitch_data[]={1,0,0,0,cos(pitch_theta),sin(pitch_theta),0,-sin(pitch_theta),cos(pitch_theta)};
        double yaw_data[]={cos(yaw_theta),0,sin(yaw_theta),0,1,0,-sin(yaw_theta),0,cos(yaw_theta)};

        Mat pitch_matrix(3,3,CV_64FC1,pitch_data);
        Mat yaw_matrix(3,3,CV_64FC1,yaw_data);
        world=pitch_matrix*yaw_matrix*tvec;
        cv2eigen(world,world_Eigen);

//        Mat R;
//
//        Rodrigues(rvec, R); //旋转矢量转换为旋转矩阵
//        Eigen::Matrix3d Rotate_M = Eigen::Matrix3d::Identity();
//        cv2eigen(R, Rotate_M);
//
//        Sophus::SO3<double> rotate(Rotate_M);
//        Eigen::Vector3d translate(tvec.ptr<double>(0)[0], tvec.ptr<double>(0)[1], tvec.ptr<double>(0)[2]);
//        armor2Camera = Sophus::SE3<double>(rotate, translate);
//
//        Eigen::Matrix3d rotation_matrix3;
//        rotation_matrix3 = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()) *
//                           Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitX()) *
//                           Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY());
//        gimbal2world = Sophus::SE3(rotation_matrix3, Eigen::Vector3d(0, 0, 0));
//
////        std::cout<<"gimbal2world"<<gimbal2world.matrix()<<std::endl;
//
//        camera2gimbal = Sophus::SE3<double>(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0, 0, 0));
//        camera2world = camera2gimbal*gimbal2world ;
//        armor2world = armor2Camera*camera2world;


//        std::cout<<"armor2Camera"<<armor2Camera.matrix()<<std::endl;
//        std::cout<<"armor2world"<<armor2world.matrix()<<std::endl;
    }

};

