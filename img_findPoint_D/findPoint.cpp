//
// Created by doudouban on 18-12-4.
//
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <fstream>
#include "findPoint.h"

using namespace cv;
using namespace std;
img_process::img_process(int i)
{
    in=i;

}
void img_process::process()
{
    colorProcess();
    depthProcess();
}

void img_process::depthProcess()
{
    char depthImg[32];
    sprintf(depthImg,"../data/depth/%d.png",in);
    Mat depth_ =imread(depthImg);
    cout<<"imread"<<endl;

//    depth_.convertTo(depth_, CV_8UC1, 32.0/255);//进行位深度的转换,2->1字节   转换为8位无符号单通道数值，然后乘32.0再除以255；
//    Mat depth = Mat::zeros(depth_.rows,depth_.cols,CV_8UC3);
//    vector<Mat> channels;
//    for (int i=0;i<3;i++)
//    {
//        channels.push_back(depth_);//进行通道数的转换,从1->3
//    }
//    merge(channels,depth);//合并三个通道，
    for (int j = 0; j < point_vet.size(); j++)
    {
        point_vet[j].x=point_vet[j].x-20;
        point_vet[j].y=point_vet[j].y+11;
        circle(depth_, point_vet[j], 2, cv::Scalar(0, 255, 0), 2);

    }
    imshow("depth",depth_);
    char resultDepthImg[32];
    sprintf(resultDepthImg,"../result/depth/%d.png",in);
    imwrite(resultDepthImg,depth_);
    waitKey();
}

void img_process::colorProcess()
{
    char colorImg[32];
    sprintf(colorImg,"../data/color/%d.png",in);
    Mat img = imread(colorImg);///4的时候有问题
    int img_r = img.rows;
    int img_c = img.cols;

    for (int i = 0; i < img_r-1; i++)
    {

        for(int j = 0; j < img_c-1; j++)
        {

            if(((img.at<Vec3b>(i,j)[2] >130)&&(img.at<Vec3b>(i,j)[2] <200))&&((img.at<Vec3b>(i,j)[1]<100)&&(img.at<Vec3b>(i,j)[1]>50))&&((img.at<Vec3b>(i,j)[0]>50)&&(img.at<Vec3b>(i,j)[0]<150)) )
            //if((img.at<Vec3b>(i,j)[2]==226)&&(img.at<Vec3b>(i,j)[1]==235)&&(img.at<Vec3b>(i,j)[0]==222))
            //if(((img.at<Vec3b>(i,j)[2] <230)&&(img.at<Vec3b>(i,j)[2] >199))&&(img.at<Vec3b>(i,j)[1]==255)&&((img.at<Vec3b>(i,j)[0]>200)&&(img.at<Vec3b>(i,j)[0]<235)) )

                //if(((img.at<Vec3b>(i,j)[2] >210)&&(img.at<Vec3b>(i,j)[2] <255))&&(img.at<Vec3b>(i,j)[1]==255)&&(img.at<Vec3b>(i,j)[0]==223) )

            {
                point_vet.push_back(Point2f(j,i));
//                cout<<"x="<<j<<endl;
//                cout<<"y="<<i<<endl;

            }
        }
    }
    cout<<"point_size="<<point_vet.size()<<endl;
    for (int j = 0; j < point_vet.size(); j++)
    {
        circle(img, point_vet[j], 2, cv::Scalar(0, 0, 0), 2);

    }
    imshow("img",img);
    char resultColorImg[32];
    sprintf(resultColorImg,"../result/color/%d.png",in);
    imwrite(resultColorImg,img);
    waitKey();

    //return img;
}
void img_process::readP(vector <Point2f> &point_Mat)
{
    char read_pointcloud_Mat[32];
    sprintf(read_pointcloud_Mat, "../data/cloud_Mat/%d.xml", in);
    FileStorage fs(read_pointcloud_Mat, FileStorage::READ);
    Mat read_3D_Mat;
    //cout<<"相机坐标："<<endl;
    fs["point3D"] >> read_3D_Mat;

    vector <Point2f> mid_pst;
    Point2f p=Point2f(0,0);
    for(int i=0;i<3;i++)
    {
        for(int j=i+1;j<4;j++)
        {
            if ((point_Mat[i].y > point_Mat[j].y))
            {
                p=point_Mat[i];
                point_Mat[i]=point_Mat[j];
                point_Mat[j]=p;
            }

        }
    }
    if(point_Mat[0].x>point_Mat[1].x)
    {
        p=point_Mat[1];
        point_Mat[1]=point_Mat[0];
        point_Mat[0]=p;
    }
    if(point_Mat[2].x>point_Mat[3].x)
    {
        p=point_Mat[3];
        point_Mat[3]=point_Mat[2];
        point_Mat[2]=p;
    }
    for(int j=0;j<4;j++)
    {
        cout<<"第 "<< j+1 << " 点坐标"<< endl <<point_Mat[j].y<<" "<<point_Mat[j].x<<endl;
        cout << read_3D_Mat.at<Vec3f>(point_Mat[j].y, point_Mat[j].x)[0] ;
        cout <<" "<< read_3D_Mat.at<Vec3f>(point_Mat[j].y, point_Mat[j].x)[1];
        cout <<" "<< read_3D_Mat.at<Vec3f>(point_Mat[j].y, point_Mat[j].x)[2] << endl;
        pst.push_back(Point3f(read_3D_Mat.at<Vec3f>(point_Mat[j].y,point_Mat[j].x)));
    }
}

