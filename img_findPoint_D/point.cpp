#include <iostream>
#include <fstream>
#include <cmath>
#include <ctime>
#include <vector>
#include <opencv2/opencv.hpp>
#include "point.h"

using namespace std;
using namespace cv;




double pointClass::GetDistance(point point1, point point2)
{
    return pow(pow(point1.x1 - point2.x1, 2) + pow(point1.x2 - point2.x2, 2) , 0.5);
}

bool pointClass::GetExist(int xm, int CentIndex[], int n)
{
    bool b = false;
    for (int i = 0; i < n; i++){
        if (xm == CentIndex[i]){
            b = true;
            break;
        }
    }
    return b;
}

pointClass::pointClass (int numOfPoint, int numOfCenter,std::vector <cv::Point2f> &point_vect)
{
    pList = new point[numOfPoint];
    centerListOld = new point[numOfCenter];
    centerListNew = new point[numOfCenter];
    point_vet.assign(point_vect.begin(),point_vect.end());
    int i = 0;
    while (i < numOfPoint){
        pList[i].x1=point_vet[i].x;
        pList[i].x2=point_vet[i].y;
        pList[i].flag = 0;
        i++;
    }

    errorNew = 0, errorOld = 0;
    for (i = 0; i < numOfCenter; i++){
        centerListOld[i].x1 = 0;
        centerListOld[i].x2 = 0;
        centerListNew[i].x1 = 0;
        centerListNew[i].x2 = 0;
        centerListNew[i].flag = 0;
        centerListOld[i].flag = 0;
    }
}

void pointClass::InitCenter(int numOfPoint, int numOfCenter, int times)
{
    int xm, i;
    int *CenterIndex = new int[numOfCenter];

    //srand((unsigned)time(0));
    srand((unsigned)times);
    for (i = 0; i < numOfCenter; i++){
        do {//防止产生重复的聚类中心
            xm = rand() % numOfPoint;
        } while (GetExist(xm, CenterIndex, i));
        CenterIndex[i] = xm;
        cout << "xm = "<< xm << endl;
    }

    for (i = 0; i < numOfCenter; i++){
        centerListOld[i] = pList[CenterIndex[i]];
    }
}


void pointClass::setPoint(int numOfPoint, int numOfCenter)
{
    errorNew = 0;
    for (int i = 0; i < numOfPoint; i++){
        int flagi = 0;
        double distance = GetDistance(pList[i], centerListOld[0]);
        for (int j = 1; j < numOfCenter; j++){
            double tmp = GetDistance(pList[i], centerListOld[j]);
            if (tmp < distance){
               // tmp = distance;  ///疑似赋值方向错误
                distance = tmp; ///llsu 修改  ？？？？？？？？？？
                flagi = j;
            }
        }
        pList[i].flag = flagi;
        //errorNew = GetDistance(pList[i], centerListOld[flagi]); ///疑似计算累计误差
        errorNew += GetDistance(pList[i], centerListOld[flagi]); ///llsu 修改计算累计误差 ！！！！！！！！！！
    }
}

bool pointClass::getError()
{
    bool b = false;
    if (errorOld != 0 && errorNew >= errorOld){
        cout << errorOld << endl;
        cout << errorNew << endl;
        b = true;
    }
    return b;
}

void pointClass::getNewCenter(int numOfPoint, int numOfCenter)
{
    for (int i = 0; i < numOfCenter; i++){
        centerListNew[i].x1 = 0;
        centerListNew[i].x2 = 0;
        centerListNew[i].flag = 0;
    }
    for (int i = 0; i < numOfPoint; i++){
        centerListNew[pList[i].flag].x1 += pList[i].x1;
        centerListNew[pList[i].flag].x2 += pList[i].x2;
        centerListNew[pList[i].flag].flag++;
    }
    for (int i = 0; i < numOfCenter; i++){
        centerListNew[i].x1 = centerListNew[i].x1 / centerListNew[i].flag;
        centerListNew[i].x2 = centerListNew[i].x2 / centerListNew[i].flag;
        centerListNew[i].flag = 0;
    }
}

void pointClass::resetCenterOld(int numOfCenter)
{
    for (int i = 0; i < numOfCenter; i++){
        centerListOld[i] = centerListNew[i];
    }
}

bool pointClass::IsEnd(int numOfCenter,bool &a)
{
    bool b = false;
    for (int i = 0; i < numOfCenter; i++){
        if (GetDistance(centerListNew[i],centerListOld[i]) > 1){
            b = true;
            errorOld = errorNew;
            break;
        }
    }
    if(!b)
    {
        if(errorOld>2000)
            a= true;
        else
            a= false;
    }
    return b;
}

void pointClass::ExportData(int numOfPoint, int numOfCenter)
{
    ofstream ofile("../result.txt");
    cout << "本次误差是：" << errorNew << endl;
    ofile << "本次误差是：" << errorNew << endl;
    for (int j = 0; j < numOfCenter; j++){
        ofile << "第" << j+1 <<"类：" << endl;
        //cout<<centerListOld[j].x1<<" "<<centerListOld[j].x2<<endl;
        point_Mat.push_back(Point2f(centerListOld[j].x1,centerListOld[j].x2));
        for (int i = 0; i < numOfPoint; i++){
            if (pList[i].flag == j){
                ofile << pList[i].x1 << " " << pList[i].x2 << endl;
            }
        }
    }
}

pointClass::~pointClass()
{
    delete[] pList;
    delete[] centerListOld;
    delete[] centerListNew;
}

