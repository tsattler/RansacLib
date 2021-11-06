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

#ifndef RANSACLIB_EXAMPLE_CALIBRATED_ABSOLUTE_POSE_ESTIMATOR_H_
#define RANSACLIB_EXAMPLE_CALIBRATED_ABSOLUTE_POSE_ESTIMATOR_H_

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

#include <PoseLib/p3p.h>
#include <PoseLib/types.h>

namespace ransac_lib {

namespace calibrated_absolute_pose {
using Eigen::Vector3d;
// An absolute pose is a Eigen 3x4 double matrix storing the rotation and
// translation of the camera.
typedef Eigen::Matrix<double, 3, 4> CameraPose;
typedef std::vector<CameraPose, Eigen::aligned_allocator<CameraPose>>
    CameraPoses;

typedef std::vector<Eigen::Vector2d> Points2D;
typedef std::vector<Vector3d, Eigen::aligned_allocator<Vector3d>> Points3D;
typedef std::vector<Vector3d, Eigen::aligned_allocator<Vector3d>> ViewingRays;

// Implements a camera pose solver for calibrated cameras. Uses the PoseLib
// implementations of the P3P solvers and Ceres for non-linear optimization.
// The P3P algorithm is used as a minimal solver. To avoid returning multiple
// models, a fourth poit is used to pick at most one model out of the up to
// four models estimated by P3P.
// The stored camera pose is [R | c], where R is the rotation from world to the
// local camera coordinate system and c is the position of the camera in the
// world coordinate system.
class CalibratedAbsolutePoseEstimator {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // The input to the constructor are the camera focal lengths f_x, f_y, a set
  // of 2D keypoint positions, and the corresponding 3D points. The 2D points
  // are expected to be centered around the principal point (i.e., the
  // principal has already been subtracted) and to be undistorted.
  // In addition, the squared inlier threshold used by *SAC is required as
  // input. It is used to pick at most one of the up to 4 solutions created by
  // the P3P solver.
  CalibratedAbsolutePoseEstimator(const double f_x, const double f_y,
                                  const double squared_inlier_threshold,
                                  const Points2D& points2D,
                                  const ViewingRays& rays,
                                  const Points3D& points3D);

  inline int min_sample_size() const { return 4; }

  inline int non_minimal_sample_size() const { return 6; }

  inline int num_data() const { return num_data_; }

  int MinimalSolver(const std::vector<int>& sample, CameraPoses* poses) const;

  // Returns 0 if no model could be estimated and 1 otherwise.
  // Implemented by non-linear optimization via Ceres.
  int NonMinimalSolver(const std::vector<int>& sample, CameraPose* pose) const;

  // Evaluates the pose on the i-th data point.
  double EvaluateModelOnPoint(const CameraPose& pose, int i) const;

  // Linear least squares solver. Calls NonMinimalSolver.
  void LeastSquares(const std::vector<int>& sample, CameraPose* pose) const;

  static void PixelsToViewingRays(const double focal_x, const double focal_y,
                                  const Points2D& points2D, ViewingRays* rays);

 protected:
  // Focal lengths in x- and y-directions.
  double focal_x_;
  double focal_y_;
  double squared_inlier_threshold_;
  // Matrix holding the 2D point positions.
  Points2D points2D_;
  // Matrix holding the corresponding 3D point positions.
  Points3D points3D_;
  // Matrix holding the viewing ray for each 2D point position.
  ViewingRays rays_;
  int num_data_;
};

}  // namespace calibrated_absolute_pose

}  // namespace ransac_lib

#endif  // RANSACLIB_EXAMPLE_CALIBRATED_ABSOLUTE_POSE_ESTIMATOR_H_
