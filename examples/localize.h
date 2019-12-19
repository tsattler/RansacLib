#include "Eigen/Eigen"

bool cameraPoseRANSAC(
    const Eigen::MatrixXd &impoints,
    const Eigen::MatrixXd &spacePoints,
    int numIterations,
    double tol,
    Eigen::Matrix<double, 3, 4> &cameraMatrix,
    int &oNumInliers);
