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

#include "generalized_calibrated_absolute_pose_estimator.h"

namespace ransac_lib {

namespace generalized_calibrated_absolute_pose {

GeneralizedCalibratedAbsolutePoseEstimator::
    GeneralizedCalibratedAbsolutePoseEstimator(
        const MultiCameraRig& rig, const double squared_inlier_threshold,
        const Points2D& points2D, const ViewingRays& rays,
        const Points3D& points3D, const std::vector<int>& camera_indices,
        const CameraPositions& positions, const CameraRotations& rotations)
    : rig_(rig),
      squared_inlier_threshold_(squared_inlier_threshold),
      points2D_(points2D),
      points3D_(points3D),
      camera_indices_(camera_indices),
      adapter_(rays, camera_indices, points3D, positions, rotations) {
  num_data_ = static_cast<int>(points2D_.size());

  rig_.global_pose.topLeftCorner<3, 3>() = Matrix3d::Identity();
  rig_.global_pose.col(3) = Vector3d::Zero();

  num_cameras_ = static_cast<int>(rig.cameras.size());
  for (int i = 0; i < num_cameras_; ++i) {
    rig_.cameras[i].pose.topLeftCorner<3, 3>() = rotations[i].transpose();
    rig_.cameras[i].pose.col(3) = positions[i];
  }
}

int GeneralizedCalibratedAbsolutePoseEstimator::MinimalSolver(
    const std::vector<int>& sample, MultiCameraRigs* poses) const {
  poses->clear();
  CameraPoses gp3p_poses = opengv::absolute_pose::gp3p(adapter_, sample);
  if (gp3p_poses.empty()) return 0;
  for (const CameraPose& pose : gp3p_poses) {
    MultiCameraRig rig;
    AssembleRig(pose, &rig);
    const double kError = EvaluateModelOnPoint(rig, sample[3]);
    if (kError < squared_inlier_threshold_) {
      poses->push_back(rig);
      // At most one pose should be correct.
      break;
    }
  }

  return static_cast<int>(poses->size());
}

// Returns 0 if no model could be estimated and 1 otherwise.
// Implemented by a simple linear least squares solver.
int GeneralizedCalibratedAbsolutePoseEstimator::NonMinimalSolver(
    const std::vector<int>& sample, MultiCameraRig* pose) const {
  CameraPose P = opengv::absolute_pose::gpnp(adapter_, sample);
  AssembleRig(P, pose);
  return 1;
}

double GeneralizedCalibratedAbsolutePoseEstimator::EvaluateModelOnPoint(
    const MultiCameraRig& pose, int i) const {
  // Transforms into the coordinate system of the rig.
  Vector3d p_r =
      pose.global_pose.topLeftCorner<3, 3>() *
          (points3D_[i] - pose.global_pose.col(3));

  // Transforms into the coordinate system of the camera.
  const Camera& cam = rig_.cameras[camera_indices_[i]];
  Vector3d p_c = cam.pose.topLeftCorner<3, 3>() * (p_r - cam.pose.col(3));

  // Check whether point projects behind the camera.
  if (p_c[2] < 0.0) return std::numeric_limits<double>::max();

  Eigen::Vector2d p_2d = p_c.head<2>() / p_c[2];
  p_2d[0] *= cam.focal_x;
  p_2d[1] *= cam.focal_y;

  return (p_2d - points2D_[i]).squaredNorm();
}

void GeneralizedCalibratedAbsolutePoseEstimator::LeastSquares(
    const std::vector<int>& sample, MultiCameraRig* pose) const {
  // OpenGV returns the transformation from the camera to the world coordinate
  // system. We store the rotation from the world to the local coordinate
  // system instead.
  Eigen::Matrix3d R = pose->global_pose.topLeftCorner<3, 3>().transpose();
  Eigen::Vector3d c = pose->global_pose.col(3);
  // At the moment, we need to copy data as we need to add the current pose
  // estimate to the adapter, which would break the requirement that this
  // function is constant.
  const int kSampleSize = static_cast<int>(sample.size());
  opengv::bearingVectors_t bearing_vectors(kSampleSize);
  opengv::points_t points(kSampleSize);
  std::vector<int> camera_indices(kSampleSize);
  for (int i = 0; i < kSampleSize; ++i) {
    const int kIdx = sample[i];
    bearing_vectors[i] = adapter_.getBearingVector(kIdx);
    points[i] = adapter_.getPoint(kIdx);
    camera_indices[i] = camera_indices_[i];
  }

  opengv::translations_t camera_positions(num_cameras_);
  opengv::rotations_t camera_rotations(num_cameras_);
  for (int i = 0; i < num_cameras_; ++i) {
    camera_positions[i] = rig_.cameras[i].pose.col(3);
    camera_rotations[i] =
        rig_.cameras[i].pose.topLeftCorner<3, 3>().transpose();
  }

  opengv::absolute_pose::NoncentralAbsoluteAdapter lsq_adapter(
      bearing_vectors, camera_indices, points, camera_positions,
      camera_rotations, c, R);

  CameraPose P = opengv::absolute_pose::optimize_nonlinear(lsq_adapter);
  AssembleRig(P, pose);
}

void GeneralizedCalibratedAbsolutePoseEstimator::AssembleRig(
    const CameraPose& pose, MultiCameraRig* rig) const {
  *rig = rig_;
  rig->global_pose.topLeftCorner<3, 3>() =
      pose.topLeftCorner<3, 3>().transpose();
  rig->global_pose.col(3) = pose.col(3);
}

}  // namespace generalized_calibrated_absolute_pose

}  // namespace ransac_lib
