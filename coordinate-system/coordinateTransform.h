#ifndef COORDINATETRANSFORM_H
#define COORDINATETRANSFORM_H
#include <cmath>
#include <cstdio>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <string>
#include <vector>

class coordinateTransform {
   private:
    Eigen::MatrixXd C1, C2, C3, A;

   public:
    Eigen::MatrixXd transformMat;
    double phi, psi, theta;
    std::vector<Eigen::Vector3d> ptsA, ptsB, ptsBcalculated;

    coordinateTransform(std::vector<Eigen::Vector3d> ptsA, std::vector<Eigen::Vector3d> ptsB);
    void dummy(std::string s);
    void calculateA();
    void calculateC1();
    void calculateC2();
    void calculateC3();
    void calculateTransformMatrix();
    void transformPointFromSysAtoSysB(Eigen::Vector3d& pt, Eigen::Vector3d& transformedPt);
    void generatePtsBcalculated();
    double errorCalculation();
};
coordinateTransform::coordinateTransform(std::vector<Eigen::Vector3d> ptsA, std::vector<Eigen::Vector3d> ptsB) {
    A = Eigen::MatrixXd(4, 4);
    C1 = Eigen::MatrixXd(4, 1);
    C2 = Eigen::MatrixXd(4, 1);
    C3 = Eigen::MatrixXd(4, 1);
    transformMat = Eigen::MatrixXd(4, 4);
    this->ptsA = ptsA;
    this->ptsB = ptsB;
}
void coordinateTransform::dummy(std::string s) {
    std::cout << s << std::endl;
}

void coordinateTransform::calculateA() {
    for (int i = 0; i < ptsA.size(); i++) {
        A(0, 0) += ptsA[i][0] * ptsA[i][0];
        A(0, 1) += ptsA[i][0] * ptsA[i][1];
        A(0, 2) += ptsA[i][0] * ptsA[i][2];
        A(0, 3) += ptsA[i][0];
        A(1, 0) += ptsA[i][0] * ptsA[i][1];
        A(1, 1) += ptsA[i][1] * ptsA[i][1];
        A(1, 2) += ptsA[i][1] * ptsA[i][2];
        A(1, 3) += ptsA[i][1];
        A(2, 0) += ptsA[i][0] * ptsA[i][2];
        A(2, 1) += ptsA[i][1] * ptsA[i][2];
        A(2, 2) += ptsA[i][2] * ptsA[i][2];
        A(2, 3) += ptsA[i][2];
        A(3, 0) += ptsA[i][0];
        A(3, 1) += ptsA[i][1];
        A(3, 2) += ptsA[i][2];
    }
    A(3, 3) = (double)ptsA.size();
}

void coordinateTransform::calculateC1() {
    for (int i = 0; i < ptsA.size(); i++) {
        C1(0, 0) += ptsB[i][0] * ptsA[i][0];
        C1(1, 0) += ptsB[i][0] * ptsA[i][1];
        C1(2, 0) += ptsB[i][0] * ptsA[i][2];
        C1(3, 0) += ptsB[i][0];
    }
}

void coordinateTransform::calculateC2() {
    for (int i = 0; i < ptsA.size(); i++) {
        C2(0, 0) += ptsB[i][1] * ptsA[i][0];
        C2(1, 0) += ptsB[i][1] * ptsA[i][1];
        C2(2, 0) += ptsB[i][1] * ptsA[i][2];
        C2(3, 0) += ptsB[i][1];
    }
}

void coordinateTransform::calculateC3() {
    for (int i = 0; i < ptsA.size(); i++) {
        C3(0, 0) += ptsB[i][2] * ptsA[i][0];
        C3(1, 0) += ptsB[i][2] * ptsA[i][1];
        C3(2, 0) += ptsB[i][2] * ptsA[i][2];
        C3(3, 0) += ptsB[i][2];
    }
}

void coordinateTransform::calculateTransformMatrix() {
    calculateA();
    calculateC1();
    calculateC2();
    calculateC3();

    // transformation matrix
    Eigen::MatrixXd row1(4, 1), row2(4, 1), row3(4, 1), temp(4, 1);
    row1 = A.inverse() * C1;
    row2 = A.inverse() * C2;
    row3 = A.inverse() * C3;

    transformMat(3, 0) = 0.0;
    transformMat(3, 1) = 0.0;
    transformMat(3, 2) = 0.0;
    transformMat(3, 3) = 1.0;

    for (int i = 0; i < 3; i++) {
        if (i == 0) temp = row1;
        if (i == 1) temp = row2;
        if (i == 2) temp = row3;
        for (int j = 0; j < 4; j++) {
            transformMat(i, j) = temp(j, 0);
        }
    }
    phi = std::atan2(transformMat(1, 2), transformMat(2, 2));
    psi = std::atan2(transformMat(0, 1), transformMat(0, 0));
    theta = std::atan2(-transformMat(0, 2), transformMat(0, 1) / std::sin(psi));
}

void coordinateTransform::transformPointFromSysAtoSysB(Eigen::Vector3d& pt, Eigen::Vector3d& transformedPt) {
    Eigen::MatrixXd tempPt(4, 1);
    tempPt << pt[0], pt[1], pt[2], 1.0;
    tempPt = transformMat * tempPt;
    transformedPt[0] = tempPt(0, 0);
    transformedPt[1] = tempPt(1, 0);
    transformedPt[2] = tempPt(2, 0);
}

void coordinateTransform::generatePtsBcalculated() {
    for (int i = 0; i < ptsA.size(); i++) {
        Eigen::Vector3d calcPt;
        transformPointFromSysAtoSysB(ptsA[i], calcPt);
        ptsBcalculated.push_back(calcPt);
    }
}

double coordinateTransform::errorCalculation() {
    generatePtsBcalculated();
    double sum = 0.0;
    for (int i = 0; i < ptsB.size(); i++) {
        Eigen::Vector3d ve = ptsB[i] - ptsBcalculated[i];
        double e = std::sqrt(ve[0] * ve[0] + ve[1] * ve[1] + ve[2] * ve[2]);
        sum += e;
    }
    return sum / ptsB.size();
}

#endif