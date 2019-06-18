//
// Created by doudouban on 18-12-16.
//
#include "Tsai.h"

Mat skew(Mat A)
{
   // CV_Assert(A.cols == 1 && A.rows == 3);
    Mat B(3, 3, CV_64FC1);
    cout<< "llsu ---> " << "2"<< endl;
    B.at<double>(0, 0) = 0.0;
    B.at<double>(0, 1) = -A.at<double>(2, 0);
    B.at<double>(0, 2) = A.at<double>(1, 0);
    cout<< "llsu ---> " << "22"<< endl;
    B.at<double>(1, 0) = A.at<double>(2, 0);
    B.at<double>(1, 1) = 0.0;
    B.at<double>(1, 2) = -A.at<double>(0, 0);

    B.at<double>(2, 0) = -A.at<double>(1, 0);
    B.at<double>(2, 1) = A.at<double>(0, 0);
    B.at<double>(2, 2) = 0.0;

    return B;
}

void Tsai_HandEye(Mat& Hcg, vector<Mat>& Hgij, vector<Mat>& Hcij)
{
    CV_Assert(Hgij.size() == Hcij.size());
    int nStatus = Hgij.size();

    Mat Rgij(3, 3, CV_64FC1);
    Mat Rcij(3, 3, CV_64FC1);

    Mat rgij(3, 1, CV_64FC1);
    Mat rcij(3, 1, CV_64FC1);

    double theta_gij;
    double theta_cij;

    Mat rngij(3, 1, CV_64FC1);
    Mat rncij(3, 1, CV_64FC1);

    Mat Pgij(3, 1, CV_64FC1);
    Mat Pcij(3, 1, CV_64FC1);

    Mat tempA(3, 3, CV_64FC1);
    Mat tempb(3, 1, CV_64FC1);

    Mat A;
    Mat b;
    Mat pinA;

    Mat Pcg_prime(3, 1, CV_64FC1);
    Mat Pcg(3, 1, CV_64FC1);
    Mat PcgTrs(1, 3, CV_64FC1);

    Mat Rcg(3, 3, CV_64FC1);
    Mat eyeM = Mat::eye(3, 3, CV_64FC1);

    Mat Tgij(3, 1, CV_64FC1);
    Mat Tcij(3, 1, CV_64FC1);

    Mat tempAA(3, 3, CV_64FC1);
    Mat tempbb(3, 1, CV_64FC1);
    Mat AA;
    Mat bb;
    Mat pinAA;

    Mat Tcg(3, 1, CV_64FC1);

    for (int i = 0; i < nStatus; i++)
    {
        ///取旋转矩阵
        Hgij[i](Rect(0, 0, 3, 3)).copyTo(Rgij);
        Hcij[i](Rect(0, 0, 3, 3)).copyTo(Rcij);

        ///由旋转矩阵转换为旋转向量
        Rodrigues(Rgij, rgij);
        Rodrigues(Rcij, rcij);

        ///求向量欧几里得范数,即2范数
        theta_gij = norm(rgij);
        theta_cij = norm(rcij);

        ///求向量的模
        rngij = rgij / theta_gij;
        rncij = rcij / theta_cij;

        ///修正的罗德里格斯参数表示姿态变化
        Pgij = 2 * sin(theta_gij / 2)*rngij;
        Pcij = 2 * sin(theta_cij / 2)*rncij;


        tempA = skew(Pgij + Pcij);

        tempb = Pcij - Pgij;

        A.push_back(tempA);
        b.push_back(tempb);
    }
    //Compute rotation
    invert(A, pinA, DECOMP_SVD);

    Pcg_prime = pinA * b;
    Pcg = 2 * Pcg_prime / sqrt(1 + norm(Pcg_prime) * norm(Pcg_prime));
    PcgTrs = Pcg.t();
    Rcg = (1 - norm(Pcg) * norm(Pcg) / 2) * eyeM + 0.5 * (Pcg * PcgTrs + sqrt(4 - norm(Pcg)*norm(Pcg))*skew(Pcg));
    //Computer Translation

    for (int i = 0; i < nStatus; i++)
    {
        Hgij[i](Rect(0, 0, 3, 3)).copyTo(Rgij);
        Hcij[i](Rect(0, 0, 3, 3)).copyTo(Rcij);

        Hgij[i](Rect(3, 0, 1, 3)).copyTo(Tgij);
        Hcij[i](Rect(3, 0, 1, 3)).copyTo(Tcij);

        tempAA = Rgij - eyeM;
        tempbb = Rcg * Tcij - Tgij;

        AA.push_back(tempAA);
        bb.push_back(tempbb);
    }

    invert(AA, pinAA, DECOMP_SVD);
    Tcg = pinAA * bb;
    Rcg.copyTo(Hcg(Rect(0, 0, 3, 3)));
    Tcg.copyTo(Hcg(Rect(3, 0, 1, 3)));

    Hcg.at<double>(3, 0) = 0.0;
    Hcg.at<double>(3, 1) = 0.0;
    Hcg.at<double>(3, 2) = 0.0;
    Hcg.at<double>(3, 3) = 1.0;
}


void Inria_HandEye(Mat& Hcg, vector<Mat>& Hgij, vector<Mat>& Hcij)
{
    CV_Assert(Hgij.size() == Hcij.size());
    int nStatus = Hgij.size();

    Mat Rgij(3, 3, CV_64FC1);
    Mat Rcij(3, 3, CV_64FC1);
    Mat Tgij(3, 1, CV_64FC1);
    Mat Tcij(3, 1, CV_64FC1);

    Mat Qgij(4, 1, CV_64FC1);
    Mat Qcij(4, 1, CV_64FC1);
    Mat Lqij(4, 4, CV_64FC1);
    Mat Rqij(4, 4, CV_64FC1);

    Mat tempA(4, 4, CV_64FC1);
    Mat A;
    Mat w, u, vt, v;
    Mat qoff(4, 1, CV_64FC1);
    Mat Roff(3, 3, CV_64FC1);

    Mat eyeM = Mat::eye(3, 3, CV_64FC1);
    Mat tempAA(3, 3, CV_64FC1);
    Mat tempbb(3, 1, CV_64FC1);
    Mat AA;
    Mat bb;
    Mat pinAA;

    Mat Toff(3, 1, CV_64FC1);

    // Compute rotation
    for (int i = 0; i < nStatus; i++)
    {
        Hgij[i](Rect(0, 0, 3, 3)).copyTo(Rgij);
        Hcij[i](Rect(0, 0, 3, 3)).copyTo(Rcij);
        Hgij[i](Rect(3, 0, 1, 3)).copyTo(Tgij);
        Hcij[i](Rect(3, 0, 1, 3)).copyTo(Tcij);

        Qgij = dcm2q(Rgij);
        Qcij = dcm2q(Rcij);

        // qA*qX=qX*qB<=>(RqA-LqB)*qx=0
        Lqij = qskewL(Qgij);
        Rqij = qskewR(Qcij);

        tempA = Lqij - Rqij;
        A.push_back(tempA);
    }

    SVD::compute(A, w, u, vt, SVD::FULL_UV);
    v = vt.t();
    v(Rect(3, 0, 1, 4)).copyTo(qoff);
    Roff = q2dcm(qoff);

    // Computer Translation
    for (int i = 0; i < nStatus; i++)
    {
        Hgij[i](Rect(0, 0, 3, 3)).copyTo(Rgij);
        Hcij[i](Rect(0, 0, 3, 3)).copyTo(Rcij);
        Hgij[i](Rect(3, 0, 1, 3)).copyTo(Tgij);
        Hcij[i](Rect(3, 0, 1, 3)).copyTo(Tcij);

        tempAA = Rgij - eyeM;
        tempbb = Roff * Tcij - Tgij;

        AA.push_back(tempAA);
        bb.push_back(tempbb);
    }

    invert(AA, pinAA, DECOMP_SVD);
    Toff = pinAA * bb;

    Roff.copyTo(Hcg(Rect(0, 0, 3, 3)));
    Toff.copyTo(Hcg(Rect(3, 0, 1, 3)));

    Hcg.at<double>(3, 0) = 0.0;
    Hcg.at<double>(3, 1) = 0.0;
    Hcg.at<double>(3, 2) = 0.0;
    Hcg.at<double>(3, 3) = 1.0;

}
