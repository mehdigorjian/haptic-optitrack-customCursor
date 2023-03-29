#include <cmath>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <vector>

#include "coordinateTransform.h"

int main(int argc, char* argv[]) {
    Eigen::Vector3d p1c, p2c, p3c, p4c, p5c, p6c,
        p1h, p2h, p3h, p4h, p5h, p6h;

    p1c = {0.208705, -64.899739, -18.780150};
    p2c = {0.208705, -64.899739, -18.780150};
    p3c = {0.208705, -64.899739, -18.780150};
    p4c = {0.208705, -64.899739, -18.780150};
    p5c = {0.208705, -64.899739, -18.780150};

    p1h = {-7.259813, 21.009457, -10.708045};
    p2h = {-7.259813, 21.009457, -10.708045};
    p3h = {-7.259813, 21.009457, -10.708045};
    p4h = {-7.259813, 21.009457, -10.708045};
    p5h = {-7.259813, 21.009457, -10.708045};

    // p1c = {0.5449, 0.1955, 0.9227};
    // p2c = {0.6862, 0.7202, 0.8004};
    // p3c = {0.8936, 0.7218, 0.2859};
    // p4c = {0.0548, 0.8778, 0.5437};
    // p5c = {0.3037, 0.5824, 0.9848};
    // p6c = {0.0462, 0.0707, 0.7157};

    // p1h = {2.5144, 7.0691, 1.9754};
    // p2h = {2.8292, 7.4454, 2.2224};
    // p3h = {3.3518, 7.3060, 2.1198};
    // p4h = {2.8392, 7.8455, 1.6229};
    // p5h = {2.4901, 7.5449, 1.9518};
    // p6h = {2.4273, 7.1354, 1.4349};

    std::vector<Eigen::Vector3d> ptsA, ptsB;
    ptsA = {p1c, p2c, p3c, p4c, p5c, p6c};
    ptsB = {p1h, p2h, p3h, p4h, p5h, p6h};

    coordinateTransform t(ptsA, ptsB);

    std::cout << "----------Transformation Matrix----------" << std::endl;
    t.calculateTransformMatrix();
    std::cout << t.transformMat << std::endl;

    std::cout << "----------Rotation Angels----------" << std::endl;
    std::cout << "phi : " << t.phi << " psi : " << t.psi << " theta : " << t.theta << std::endl;

    std::cout << "----------Error----------" << std::endl;
    double ee = t.errorCalculation();
    std::cout << ee << std::endl;

    return 0;
}