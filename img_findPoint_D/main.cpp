#include <vector>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include "kmeans.h"
#include "findPoint.h"
#include "get_Rt.h"
#include <pcl/io/pcd_io.h>//Eigen是PCL的第三方
#include "Tsai.h"

using namespace std;
using namespace cv;

void pose_estimation_3d3d ( const vector<Point3f>& pts1, const vector<Point3f>& pts2, Mat& R, Mat& t ,Mat&Rt);

int main()
{

    ///建立变量
    int n,m;
    int loop;
    Mat R,t,Rt;
    vector<Mat> Hgij,Hcij;
    Mat Hcg(4,4,CV_64FC1);
//    Mat R2= (Mat_<float> ( 3,3 ) <<
//                                 0,1,0,
//            -1,0,0,
//            0,0,1);



    ifstream infile;
    infile.open("../data.txt");
    infile>>loop;
    for( int i=0;i<loop;i++)
    {
        infile>>n;
        infile>>m;
        img_process p1(n);
        p1.process();
        kMeans k1(p1.point_vet.size(),4,p1.point_vet);
        k1.k_means();
        p1.readP(k1.point_Mat);///得到第一张图片中点的三维信息
        img_process p2(m);
        p2.process();
        kMeans k2(p2.point_vet.size(),4,p2.point_vet);
        k2.k_means();
        p2.readP(k2.point_Mat);///得到第二张图片中点的三维信息

        pose_estimation_3d3d(p1.pst,p2.pst,R,t,Rt);///算出两张图片中的相机坐标系之间的Rt;
        cout<<"R ="<<endl<<R<<endl;
        cout<<"t ="<<endl<<t<<endl;
        cout<<"Rt"<<endl<<Rt<<endl;
        Mat temp;
        Rt.convertTo(temp, CV_64FC1);
        Hcij.push_back(temp);
    }

    infile.close();

    ifstream inDATA;
    inDATA.open("../angle.txt");

    for(int i=0;i<loop;i++)
    {
        get_Rt world_Rt;
        inDATA>>world_Rt.angle_X>>world_Rt.angle_Y>>world_Rt.angle_Z;
/*        cout<<"依次输入绕X轴、Y轴、Z轴旋转的角度"<<endl;
        cin>>world_Rt.angle_X>>world_Rt.angle_Y>>world_Rt.angle_Z;*/
        world_Rt.rotate_Rt();
        cout<<"世界坐标系之间的Rt"<<world_Rt.Rt<<endl;
        Mat temp;
        world_Rt.Rt.convertTo(temp, CV_64FC1);
        Hgij.push_back(temp);

    }
    inDATA.close();

    Tsai_HandEye(Hcg, Hgij, Hcij);
    //Inria_HandEye(Hcg, Hgij, Hcij);
    cout << "Hcg = "<< endl << Hcg << endl;
	return 0;
}



void pose_estimation_3d3d ( const vector<Point3f>& pts1, const vector<Point3f>& pts2, Mat& R, Mat& t, Mat& Rt)
{
    Point3f p1, p2;     // center of mass//质心
    int N = pts1.size();
    for ( int i=0; i<N; i++ )
    {
        p1 += pts1[i];
        p2 += pts2[i];
    }
    p1 = Point3f( Vec3f(p1) / N);
    p2 = Point3f( Vec3f(p2) / N);
    vector<Point3f>     q1 ( N ), q2 ( N ); // remove the center//去质心
    for ( int i=0; i<N; i++ )
    {
        q1[i] = pts1[i] - p1;
        q2[i] = pts2[i] - p2;
    }

    // compute q1*q2^T
    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for ( int i=0; i<N; i++ )
    {
        W += Eigen::Vector3d ( q1[i].x, q1[i].y, q1[i].z ) * Eigen::Vector3d ( q2[i].x, q2[i].y, q2[i].z ).transpose();
    }
//    cout<<"W="<<W<<endl;

    // SVD on W
    Eigen::JacobiSVD<Eigen::Matrix3d> svd ( W, Eigen::ComputeFullU|Eigen::ComputeFullV );
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
//    cout<<"U="<<U<<endl;
//    cout<<"V="<<V<<endl;

    Eigen::Matrix3d R_ = U* ( V.transpose() );
    Eigen::Vector3d t_ = Eigen::Vector3d ( p1.x, p1.y, p1.z ) - R_ * Eigen::Vector3d ( p2.x, p2.y, p2.z );

    // Matrix3d convert to cv::Mat
    R = ( Mat_<float> ( 3,3 ) <<
                              R_ ( 0,0 ), R_ ( 0,1 ), R_ ( 0,2 ),
            R_ ( 1,0 ), R_ ( 1,1 ), R_ ( 1,2 ),
            R_ ( 2,0 ), R_ ( 2,1 ), R_ ( 2,2 )
    );
    // Vector3d convert to cv::Mat
    t = ( Mat_<float> ( 3,1 ) << t_ ( 0,0 ), t_ ( 1,0 ), t_ ( 2,0 ) );
    Rt = (Mat_<float> (4,4)<<
            R.at<float>(0,0),R.at<float>(0,1),R.at<float>(0,2),t.at<float>(0,0),
            R.at<float>(1,0),R.at<float>(1,1),R.at<float>(1,2),t.at<float>(1,0),
            R.at<float>(2,0),R.at<float>(2,1),R.at<float>(2,2),t.at<float>(2,0),
                    0,0,0,1
            );
}