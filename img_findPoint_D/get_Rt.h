//
// Created by doudouban on 18-12-13.
//

#ifndef IMG_FINDPOINT_D_GET_RT_H
#define IMG_FINDPOINT_D_GET_RT_H



#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#define Pi 3.1415926
using namespace cv;
class get_Rt{
protected:
    void rotate_Rt_X();
    void rotate_Rt_Y();
    void rotate_Rt_Z();
    Mat Rz= (Mat_<float> ( 3,3 ) <<
                                 0,0,0,
            0,0,0,
            0,0,0);
    Mat Rx= (Mat_<float> ( 3,3 ) <<
                                 0,0,0,
            0,0,0,
            0,0,0);
    Mat Ry= (Mat_<float> ( 3,3 ) <<
                                 0,0,0,
            0,0,0,
            0,0,0);
public:
    get_Rt ();
    double angle_X,angle_Y,angle_Z;
    void rotate_Rt ();
    Mat Rt=(Mat_<float>(4,4)<<
            0,0,0,0,
            0,0,0,0,
            0,0,0,0,
            0,0,0,1
            );
    ~get_Rt ();


};
#endif //IMG_FINDPOINT_D_GET_RT_H
