//
// Created by doudouban on 18-12-13.
//
#include "get_Rt.h"
#include <opencv2/opencv.hpp>
get_Rt::get_Rt() {}
void get_Rt::rotate_Rt ()
{
    rotate_Rt_X();
    rotate_Rt_Y();
    rotate_Rt_Z();
    Mat R= (Mat_<float> ( 3,3 ) <<
                                 0,0,0,
            0,0,0,
            0,0,0);
    Mat R1= (Mat_<float> ( 3,3 ) <<
                                0,-1,0,
            1,0,0,
            0,0,1);
    Mat R2= (Mat_<float> ( 3,3 ) <<
                                0,1,0,
            -1,0,0,
            0,0,1);

    R=Rz*Ry*Rx;
    Rt=(Mat_<float>(4,4) <<
            R.at<float>(0,0),R.at<float>(0,1),R.at<float>(0,2),0,
            R.at<float>(1,0),R.at<float>(1,1),R.at<float>(1,2),0,
            R.at<float>(2,0),R.at<float>(2,1),R.at<float>(2,2),0,
            0,0,0,1
    );

}
void get_Rt::rotate_Rt_Z()
{

    Rz.at<float>(0,0)=cos(angle_Z*Pi/180);
    Rz.at<float>(0,1)=-sin(angle_Z*Pi/180);
    Rz.at<float>(1,0)=sin(angle_Z*Pi/180);
    Rz.at<float>(1,1)=cos(angle_Z*Pi/180);
    Rz.at<float>(2,2)=1;


}
void get_Rt::rotate_Rt_X()
{

    Rx.at<float>(0,0)=1;
    Rx.at<float>(1,1)=cos(angle_X*Pi/180);
    Rx.at<float>(1,2)=-sin(angle_X*Pi/180);
    Rx.at<float>(2,1)=sin(angle_X*Pi/180);
    Rx.at<float>(2,2)=cos(angle_X*Pi/180);
}
void get_Rt::rotate_Rt_Y()
{

    Ry.at<float>(1,1)=1;
    Ry.at<float>(0,0)=cos(angle_Y*Pi/180);
    Ry.at<float>(0,2)=sin(angle_Y*Pi/180);
    Ry.at<float>(2,0)=-sin(angle_Y*Pi/180);
    Ry.at<float>(2,2)=cos(angle_Y*Pi/180);
}
get_Rt::~get_Rt() {}