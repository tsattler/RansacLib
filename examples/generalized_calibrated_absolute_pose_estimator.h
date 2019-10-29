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

#ifndef RANSACLIB_EXAMPLE_GENERALIZED_CALIBRATED_ABSOLUTE_POSE_ESTIMATOR_H_
#define RANSACLIB_EXAMPLE_GENERALIZED_CALIBRATED_ABSOLUTE_POSE_ESTIMATOR_H_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <random>
#include <vector>

#include <Eigen/Core>
#include <Eigen/StdVector>

#include <opengv/absolute_pose/NoncentralAbsoluteAdapter.hpp>
#include <opengv/absolute_pose/methods.hpp>
#include <opengv/types.hpp>

namespace ransac_lib {

namespace generalized_calibrated_absolute_pose {
using Eigen::aligned_allocator;
using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;
// An absolute pose is a Eigen 3x4 double matrix storing the rotation and
// translation of the camera.
typedef Eigen::Matrix<double, 3, 4> CameraPose;
typedef std::vector<CameraPose, aligned_allocator<CameraPose>> CameraPoses;

// Defines a camera with intrinsics and extrinsics.
// We use a simple camera model, where we assume that all 3D points have
// already been centered around the principal point and distortion has been
// removed from the image.
struct Camera {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CameraPose pose;
  double focal_x;
  double focal_y;
};
typedef std::vector<Camera, aligned_allocator<Camera>> Cameras;

// Defines a multi-camera rig as a set of individual cameras. The pose of each
// camera is given by the relative pose of the camera wrt. to a local camera
// coordinate system. In addition, the global pose of the rig (as [R, c]) is
// also stored.
struct MultiCameraRig {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Cameras cameras;
  CameraPose global_pose;
};
typedef std::vector<MultiCameraRig, aligned_allocator<MultiCameraRig>>
    MultiCameraRigs;

typedef std::vector<Vector2d, aligned_allocator<Vector2d>> Points2D;
typedef std::vector<Vector3d, aligned_allocator<Vector3d>> Points3D;
typedef std::vector<Vector3d, aligned_allocator<Vector3d>> ViewingRays;

typedef std::vector<Vector3d, aligned_allocator<Vector3d>> CameraPositions;
typedef std::vector<Matrix3d, aligned_allocator<Matrix3d>> CameraRotations;

// Implements a camera pose solver for generalized calibrated cameras. Uses
// the OpenGV implementations of the GP3P and GPnP solvers, as well as for
// non-linear optimization.
// The GP3P algorithm is used as a minimal solver. To avoid returning
// multiple models, a fourth poit is used to pick at most one model out of
// the up to four models estimated by GP4P. The output is the absolute pose
// of each pose in the multi-camera rig in world coordinates, stored as [R |
// c], where R is the rotation from world to the local camera coordinate
// system and c is the position of the camera in the world coordinate
// system.
class GeneralizedCalibratedAbsolutePoseEstimator {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // The input to the constructor is a set of cameras that define a multi-camera
  // rig, a set of 2D keypoint positions, the corresponding 3D points, the
  // corresponding viewing rays for each 2D keypoint (defined as the viewing ray
  // of the point in its corresponding camera), and the camera index for each
  // match. The 2D points are expected to be centered around the principal point
  // (i.e., the principal has already been subtracted) and to be undistorted.
  // The poses of the cameras in the multi-camera rig are defined by the input
  // parameters positions and rotations. positions stores the camera centers in
  // the rig coordinate system while rotations stores the rotations from the
  // local camera coordinates to the rig coordinate system (as per OpenGV
  // convention). From the input rig, only the intrinsics are used. The
  // constructor updates the extrinsics of the rig cameras such that they store
  // the camera center in rig coordinates and the rotation from rig coordinates
  // to local camera coordinates.
  // In addition, the squared inlier threshold used by *SAC is required as
  // input. It is used to pick at most one of the up to 4 solutions created by
  // the GP3P solver.
  GeneralizedCalibratedAbsolutePoseEstimator(
      const MultiCameraRig& rig, const double squared_inlier_threshold,
      const Points2D& points2D, const ViewingRays& rays,
      const Points3D& points3D, const std::vector<int>& camera_indices,
      const CameraPositions& positions, const CameraRotations& rotations);

  inline int min_sample_size() const { return 4; }

  // OpenGV's GPnP implementation asserts that there are at least 6 matches as
  // input (to ensure that only a single solution is provided).
  inline int non_minimal_sample_size() const { return 6; }

  inline int num_data() const { return num_data_; }

  int MinimalSolver(const std::vector<int>& sample,
                    MultiCameraRigs* poses) const;

  // Returns 0 if no model could be estimated and 1 otherwise.
  // Implemented by a simple linear least squares solver.
  int NonMinimalSolver(const std::vector<int>& sample,
                       MultiCameraRig* pose) const;

  // Evaluates the model on the i-th data point.
  double EvaluateModelOnPoint(const MultiCameraRig& pose, int i) const;

  // Linear least squares solver. Calls NonMinimalSolver.
  void LeastSquares(const std::vector<int>& sample, MultiCameraRig* pose) const;

 protected:
  // Given the pose estimated by OpenGV, creates a multi-camera rig that
  // contains the absolute pose of each camera (in the form [R, c]).
  void AssembleRig(const CameraPose& pose, MultiCameraRig* rig) const;

  MultiCameraRig rig_;
  double squared_inlier_threshold_;
  // Matrix holding the 2D point positions.
  Points2D points2D_;
  // Matrix holding the corresponding 3D point positions.
  Points3D points3D_;
  // For each match, stores the camera in the multi-camera rig where it was
  // detected.
  std::vector<int> camera_indices_;
  // The adapter used by OpenGV's solvers.
  opengv::absolute_pose::NoncentralAbsoluteAdapter adapter_;
  int num_data_;
  int num_cameras_;
};

}  // namespace generalized_calibrated_absolute_pose

}  // namespace ransac_lib

#endif  // RANSACLIB_EXAMPLE_GENERALIZED_CALIBRATED_ABSOLUTE_POSE_ESTIMATOR_H_
