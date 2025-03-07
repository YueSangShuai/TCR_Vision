#pragma once
#include<opencv2/opencv.hpp>
using namespace cv;

//步兵、工程        英雄
enum ArmorType {
    SMALL = 0, LARGE = 1
};


struct alignas(4) bbox_t {  //装甲板
    cv::Point2f pts[4]; // [pt0, pt1, pt2, pt3]  左上 左下 右下 右上
    float confidence;
    int color_id; // 0: blue, 1: red, 2: gray
    int tag_id;   // 0: guard, 1-5: number, 6: base
    int type; //0: small, 1:large

};