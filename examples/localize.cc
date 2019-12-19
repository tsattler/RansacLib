#include <iostream>
#include <RansacLib/ransac.h>
#include "calibrated_absolute_pose_estimator.h"
#include "localize.h"

#define NPTS 100
bool test_ransac()
{
    int n = 1000;
    Eigen::Matrix<double,3,NPTS> X = Eigen::MatrixXd::Random(3,NPTS)*10.0;
    for (int k=0; k<NPTS; ++k)
    {
        X(2,k) += 25.0;
    }
    Eigen::Matrix<double,4,NPTS> U = Eigen::MatrixXd::Ones(4,NPTS);
    U.block(0,0,3,NPTS) = X;
    Eigen::Vector3d vaa = Eigen::Vector3d::Random(3,1)*0.2;
    Eigen::AngleAxis<double> aa(vaa.norm(), vaa / vaa.norm());
    Eigen::Matrix<double,3,4> P_gt = Eigen::MatrixXd::Random(3,4);
    P_gt.block(0,0,3,3) = aa.toRotationMatrix();
    Eigen::Matrix<double,3,NPTS> x = P_gt*U;
    Eigen::Matrix<double,3,4> P = Eigen::MatrixXd::Zero(3,4);

    bool res = cameraPoseRANSAC(x, X, 1000, 0.008, P, n);
    return (res && n==NPTS && (P-P_gt).norm()<1e-8);
}

bool cameraPoseRANSAC(const Eigen::MatrixXd &impoints, const Eigen::MatrixXd &spacePoints, int numIterations, double tol, Eigen::Matrix<double,3,4> &cameraMatrixOut, int &numInliersOut)
{
    numInliersOut = 0;

    using ransac_lib::LocallyOptimizedMSAC;
    using ransac_lib::calibrated_absolute_pose::CalibratedAbsolutePoseEstimator;
    using ransac_lib::calibrated_absolute_pose::CameraPose;
    using ransac_lib::calibrated_absolute_pose::CameraPoses;
    using ransac_lib::calibrated_absolute_pose::Points2D;
    using ransac_lib::calibrated_absolute_pose::Points3D;

    const int kNumMatches = impoints.cols();
    if (kNumMatches <= 3)
    {
        return false;
    }
    if (spacePoints.cols() != kNumMatches)
    {
        return false;
    }
    if (impoints.rows() != 3)
    {
        return false;
    }
    if (spacePoints.rows() != 3)
    {
        return false;
    }

    Points2D points2D;
    Points3D points3D;

    for (size_t k = 0; k < kNumMatches; ++k)
    {
        points2D.push_back(impoints.block(0,k,2,1)/impoints(2,k));
        points3D.push_back(spacePoints.block(0,k,3,1));
    }

    opengv::bearingVectors_t rays;

    double focal_x = 1.0, focal_y = 1.0; //Normalized cameras as input
    //Could read this directly from the input instead...
    CalibratedAbsolutePoseEstimator::PixelsToViewingRays(
        focal_x, focal_y, points2D, &rays);

    ransac_lib::LORansacOptions options;
    options.min_num_iterations_ = 100u;
    options.max_num_iterations_ = numIterations;
    options.min_sample_multiplicator_ = 7;
    options.num_lsq_iterations_ = 4;
    options.num_lo_steps_ = 10;
    options.lo_starting_iterations_ = 20;
    options.final_least_squares_ = true;

    std::random_device rand_dev;
    options.random_seed_ = rand_dev();

    options.squared_inlier_threshold_ = tol*tol;

    CalibratedAbsolutePoseEstimator solver(
        focal_x, focal_y, options.squared_inlier_threshold_,
        points2D, rays, points3D);

    LocallyOptimizedMSAC<CameraPose, CameraPoses,
                         CalibratedAbsolutePoseEstimator> lomsac;

    ransac_lib::RansacStatistics ransac_stats;
    CameraPose best_model = Eigen::MatrixXd::Zero(3,4);
    best_model(0,0) = 1.0;
    best_model(1,1) = 1.0;
    best_model(2,2) = 1.0;

    //This line causes a crash in Matlab...
    numInliersOut = lomsac.EstimateModel(options, solver, &best_model, &ransac_stats);
    best_model.block(0,3,3,1) = -best_model.block(0,0,3,3)*best_model.block(0,3,3,1);
    cameraMatrixOut = best_model;
    return numInliersOut >= 4;
}
