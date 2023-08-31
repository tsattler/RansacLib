// Copyright (c) 2019, Torsten Sattler
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//
//     * Neither the name of the copyright holder nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// author: Torsten Sattler, torsten.sattler.de@googlemail.com

#include <iostream>

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/StdVector>

#include "calibrated_absolute_pose_estimator.h"

namespace ransac_lib {

namespace calibrated_absolute_pose {

struct ReprojectionError {
  ReprojectionError(double x, double y, double X, double Y, double Z,
                    double fx, double fy)
      : point2D_x(x),
        point2D_y(y),
        point3D_X(X),
        point3D_Y(Y),
        point3D_Z(Z),
        f_x(fx),
        f_y(fy) {}

  template <typename T>
  bool operator()(const T* const camera, T* residuals) const {
    // The last three entries are the camera position.
    T p[3];
    p[0] = point3D_X - camera[3];
    p[1] = point3D_Y - camera[4];
    p[2] = point3D_Z - camera[5];

    // The first three entries correspond to the rotation matrix stored in an
    // angle-axis representation.
    T p_rot[3];
    ceres::AngleAxisRotatePoint(camera, p, p_rot);

    T x_proj = static_cast<T>(f_x) * p_rot[0] / p_rot[2];
    T y_proj = static_cast<T>(f_y) * p_rot[1] / p_rot[2];

    residuals[0] = static_cast<T>(point2D_x) - x_proj;
    residuals[1] = static_cast<T>(point2D_y) - y_proj;

    return true;
  }

  // Factory function
  static ceres::CostFunction* CreateCost(const double x, const double y,
                                         const double X, const double Y,
                                         const double Z, const double fx,
                                         const double fy) {
    return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, 6>(
        new ReprojectionError(x, y, X, Y, Z, fx, fy)));
  }

  // Assumes that the measurement is centered around the principal point.
  // This camera model does not take any radial distortion into account. If
  // radial distortion is present, one should undistort the measurements first.
  double point2D_x;
  double point2D_y;
  // The 3D point position is fixed as we are only interested in refining the
  // camera parameters.
  double point3D_X;
  double point3D_Y;
  double point3D_Z;
  double f_x;
  double f_y;
};

CalibratedAbsolutePoseEstimator::CalibratedAbsolutePoseEstimator(
    const double f_x, const double f_y, const double squared_inlier_threshold,
    const Points2D& points2D, const ViewingRays& rays, const Points3D& points3D)
    : focal_x_(f_x),
      focal_y_(f_y),
      squared_inlier_threshold_(squared_inlier_threshold),
      points2D_(points2D),
      points3D_(points3D),
      rays_(rays) {
  num_data_ = static_cast<int>(points2D_.size());
}

int CalibratedAbsolutePoseEstimator::MinimalSolver(
    const std::vector<int>& sample, CameraPoses* poses) const {
  poses->clear();
  std::vector<Eigen::Vector3d> x(3), X(3);
  for (int i = 0; i < 3; ++i) {
    x[i] = rays_[sample[i]];
    X[i] = points3D_[sample[i]];
  }

  std::vector<poselib::CameraPose> poselib_poses;
  int num_sols = poselib::p3p(x, X, &poselib_poses);
  if (num_sols == 0) return 0;

  for (const poselib::CameraPose& pose : poselib_poses) {
    CameraPose P;
    P.topLeftCorner<3, 3>() = pose.R();
    P.col(3) = -pose.R().transpose() * pose.t;

    const double kError = EvaluateModelOnPoint(P, sample[3]);
    if (kError < squared_inlier_threshold_) {
      poses->push_back(P);
      break;
    }
  }

  return static_cast<int>(poses->size());
}

// Returns 0 if no model could be estimated and 1 otherwise.
// Implemented via non-linear optimization in Ceres.
int CalibratedAbsolutePoseEstimator::NonMinimalSolver(
    const std::vector<int>& sample, CameraPose* pose) const {
  CameraPoses poses;
  if (MinimalSolver(sample, &poses) == 1) {
    *pose = poses[0];
    LeastSquares(sample, pose);
    return 1;
  } else {
    return 0;
  }
}

// Evaluates the pose on the i-th data point.
double CalibratedAbsolutePoseEstimator::EvaluateModelOnPoint(
    const CameraPose& pose, int i) const {
  Eigen::Vector3d p_c =
      pose.topLeftCorner<3, 3>() * (points3D_[i] - pose.col(3));

  // Check whether point projects behind the camera.
  if (p_c[2] < 0.0) return std::numeric_limits<double>::max();

  Eigen::Vector2d p_2d = p_c.head<2>() / p_c[2];
  p_2d[0] *= focal_x_;
  p_2d[1] *= focal_y_;

  return (p_2d - points2D_[i]).squaredNorm();
}

// Reference implementation using Ceres for refinement.
void CalibratedAbsolutePoseEstimator::LeastSquares(
    const std::vector<int>& sample, CameraPose* pose) const {
  Eigen::AngleAxisd aax(pose->topLeftCorner<3, 3>());
  Eigen::Vector3d aax_vec = aax.axis() * aax.angle();
  double camera[6];
  camera[0] = aax_vec[0];
  camera[1] = aax_vec[1];
  camera[2] = aax_vec[2];
  camera[3] = pose->col(3)[0];
  camera[4] = pose->col(3)[1];
  camera[5] = pose->col(3)[2];

  ceres::Problem refinement_problem;

  const int kSampleSize = static_cast<int>(sample.size());
  for (int i = 0; i < kSampleSize; ++i) {
    const int kIdx = sample[i];
    const Eigen::Vector2d& p_img = points2D_[kIdx];
    const Eigen::Vector3d& p_3D = points3D_[kIdx];
    ceres::CostFunction* cost_function =
        ReprojectionError::CreateCost(p_img[0], p_img[1], p_3D[0], p_3D[1],
                                      p_3D[2], focal_x_, focal_y_);

    refinement_problem.AddResidualBlock(cost_function, nullptr, camera);
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = false;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &refinement_problem, &summary);

  if (summary.IsSolutionUsable()) {
    Eigen::Vector3d axis(camera[0], camera[1], camera[2]);
    double angle = axis.norm();
    axis.normalize();
    aax.axis() = axis;
    aax.angle() = angle;

    pose->topLeftCorner<3, 3>() = aax.toRotationMatrix();
    pose->col(3) = Eigen::Vector3d(camera[3], camera[4], camera[5]);
  }
}

void CalibratedAbsolutePoseEstimator::PixelsToViewingRays(
    const double focal_x, const double focal_y, const Points2D& points2D,
    ViewingRays* rays) {
  const int kNumData = static_cast<int>(points2D.size());

  // Creates the bearing vectors and points for the OpenGV adapter.
  rays->resize(kNumData);
  for (int i = 0; i < kNumData; ++i) {
    (*rays)[i] = points2D[i].homogeneous();
    (*rays)[i][0] /= focal_x;
    (*rays)[i][1] /= focal_y;
    (*rays)[i].normalize();
  }
}
}  // namespace calibrated_absolute_pose

}  // namespace ransac_lib
