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

#include <Eigen/StdVector>

#include "calibrated_absolute_pose_estimator.h"

namespace ransac_lib {

namespace calibrated_absolute_pose {

CalibratedAbsolutePoseEstimator::CalibratedAbsolutePoseEstimator(
    const double f_x, const double f_y, const double squared_inlier_threshold,
    const Points2D& points2D, const ViewingRays& rays, const Points3D& points3D)
    : focal_x_(f_x),
      focal_y_(f_y),
      squared_inlier_threshold_(squared_inlier_threshold),
      points2D_(points2D),
      points3D_(points3D),
      adapter_(rays, points3D) {
  num_data_ = static_cast<int>(points2D_.size());
}

int CalibratedAbsolutePoseEstimator::MinimalSolver(
    const std::vector<int>& sample, CameraPoses* poses) const {
  poses->clear();
  CameraPoses p3p_poses = opengv::absolute_pose::p3p_kneip(adapter_, sample);
  if (p3p_poses.empty()) return 0;
  for (const CameraPose& pose : p3p_poses) {
    CameraPose P = pose;
    // OpenGV returns the transformation from the camera to the world coordinate
    // system. We store the rotation from the world to the local coordinate
    // system instead.
    P.topLeftCorner<3, 3>() = pose.topLeftCorner<3, 3>().transpose();
    const double kError = EvaluateModelOnPoint(P, sample[3]);
    if (kError < squared_inlier_threshold_) {
      poses->push_back(P);
      // At most one pose should be correct.
      break;
    }
  }

  return static_cast<int>(poses->size());
}

// Returns 0 if no model could be estimated and 1 otherwise.
// Implemented by a simple linear least squares solver.
int CalibratedAbsolutePoseEstimator::NonMinimalSolver(
    const std::vector<int>& sample, CameraPose* pose) const {
  CameraPose P = opengv::absolute_pose::epnp(adapter_, sample);
  // OpenGV returns the transformation from the camera to the world coordinate
  // system. We store the rotation from the world to the local coordinate
  // system instead.
  *pose = P;
  pose->topLeftCorner<3, 3>() = P.topLeftCorner<3, 3>().transpose();
  return 1;
}

// Evaluates the line on the i-th data point.
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

// Linear least squares solver. Calls NonMinimalSolver.
void CalibratedAbsolutePoseEstimator::LeastSquares(
    const std::vector<int>& sample, CameraPose* pose) const {
  // OpenGV returns the transformation from the camera to the world coordinate
  // system. We store the rotation from the world to the local coordinate
  // system instead.
  Eigen::Matrix3d R = pose->topLeftCorner<3, 3>().transpose();
  Eigen::Vector3d c = pose->col(3);
  // At the moment, we need to copy data as we need to add the current pose
  // estimate to the adapter, which would break the requirement that this
  // function is constant.
  const int kSampleSize = static_cast<int>(sample.size());
  opengv::bearingVectors_t bearing_vectors(kSampleSize);
  opengv::points_t points(kSampleSize);
  for (int i = 0; i < kSampleSize; ++i) {
    const int kIdx = sample[i];
    bearing_vectors[i] = adapter_.getBearingVector(kIdx);
    points[i] = adapter_.getPoint(kIdx);
  }

  opengv::absolute_pose::CentralAbsoluteAdapter lsq_adapter(bearing_vectors,
                                                            points, c, R);

  CameraPose P = opengv::absolute_pose::optimize_nonlinear(lsq_adapter);
  *pose = P;
  pose->topLeftCorner<3, 3>() = P.topLeftCorner<3, 3>().transpose();
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
