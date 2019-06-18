//
// Created by doudouban on 18-12-4.
//
#include "kmeans.h"
#include "point.h"
using namespace std;

kMeans::kMeans(int p,int c,std::vector<cv::Point2f>point)
{
    numOfPoint=p;
    numOfCenter=c;
    point_vect.assign(point.begin(),point.end());
}

void kMeans::ret_point_Mat(std::vector<cv::Point2f>&point)
{

    point_Mat.assign(point.begin(),point.end());
}

void kMeans::k_means()
{

    bool a= false;
    int times = 1000;
    do{
        times++;
        pointClass pc(numOfPoint, numOfCenter,point_vect);
        pc.InitCenter(numOfPoint, numOfCenter, times);
        bool b = true;
        while (b){
            pc.setPoint(numOfPoint, numOfCenter);
            if (pc.getError()){
                a= true;
                break;
            }
            pc.getNewCenter(numOfPoint, numOfCenter);
            b = pc.IsEnd(numOfCenter,a);
            pc.resetCenterOld(numOfCenter);
        }
        if (b) {
            cout << "本次聚类操作无法完成！！" << endl;
        } else if(a){
            cout<<"本次误差不满足要求"<<endl;
        }else{
            pc.ExportData(numOfPoint, numOfCenter);
            ret_point_Mat(pc.point_Mat);
        }
    }while (a);


}

