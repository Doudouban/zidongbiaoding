//
// Created by doudouban on 18-12-4.
//

#ifndef IMG_FINDPOINT_FINDPOINT_H
#define IMG_FINDPOINT_FINDPOINT_H

#include <vector>
#include <opencv2/opencv.hpp>
#include <iostream>

class img_process
{
protected:
    int in;
    void depthProcess();///深度图上画出特征点
    void colorProcess();///彩图上画出特征点
public:
    std::vector <cv::Point2f> point_vet;
    std::vector <cv::Point2f> point_Mat;
    std::vector <cv::Point3f> pst;
    img_process(int i);
    void readP(std::vector <cv::Point2f> &point_Mat);
    void process();
};

#endif //IMG_FINDPOINT_FINDPOINT_H
