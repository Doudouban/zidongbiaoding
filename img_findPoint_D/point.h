//
// Created by doudouban on 18-12-4.
//

#ifndef IMG_FINDPOINT_POINT_H
#define IMG_FINDPOINT_POINT_H

#include <vector>
#include <opencv2/opencv.hpp>
struct point{
    double x1, x2;
    int flag;
};

class pointClass{
protected:
    double errorOld, errorNew;
    std::vector <cv::Point2f> point_vet;
    point *pList, *centerListNew, *centerListOld;
    double GetDistance(point point1, point point2);
    bool GetExist(int xm, int centerList[], int n);
public:
    std::vector <cv::Point2f> point_Mat;
    pointClass (int numOfPoint, int numOfCenter,std::vector <cv::Point2f> &point_vect);
    void InitCenter (int numOfPoint, int numOfCenter, int times);
    void setPoint (int numOfPoint, int numOfCenter);
    bool getError();
    void getNewCenter (int numOfPoint, int numOfCenter);
    bool IsEnd (int numOfCenter,bool &a);
    void ExportData (int numOfPoint, int numOfCenter);
    void resetCenterOld (int numOfCenter);
    ~pointClass ();
};
#endif //IMG_FINDPOINT_POINT_H
