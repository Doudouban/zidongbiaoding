//
// Created by doudouban on 18-12-4.
//

#ifndef IMG_FINDPOINT_KMEANS_H
#define IMG_FINDPOINT_KMEANS_H
#include <vector>
#include <opencv2/opencv.hpp>

class kMeans{
protected:
    int numOfPoint, numOfCenter;
    std::vector<cv::Point2f> point_vect;
    void ret_point_Mat(std::vector<cv::Point2f>&);
public:
    std::vector<cv::Point2f> point_Mat;
    kMeans(int,int,std::vector<cv::Point2f>);

    void k_means();
};
#endif //IMG_FINDPOINT_KMEANS_H
