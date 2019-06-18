//
// Created by doudouban on 18-12-18.
//

#ifndef IMG_FINDPOINT_TSAI_H
#define IMG_FINDPOINT_TSAI_H
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include "quaternion.h"
using namespace std;
using namespace cv;

Mat skew(Mat A);
void Tsai_HandEye(Mat& Hcg, vector<Mat>& Hgij, vector<Mat>& Hcij);
void Inria_HandEye(Mat& Hcg, vector<Mat>& Hgij, vector<Mat>& Hcij);

#endif //IMG_FINDPOINT_TSAI_H
