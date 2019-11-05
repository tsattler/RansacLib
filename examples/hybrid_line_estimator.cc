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

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <random>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include "hybrid_line_estimator.h"

namespace ransac_lib {

HybridLineEstimator::HybridLineEstimator(
    const Eigen::Matrix2Xd& points, const Eigen::Matrix4Xd& points_with_normals,
    const std::vector<double>& prior_probabilities) {
  points_ = points;
  num_points_ = points_.cols();
  points_with_normals_ = points_with_normals;
  num_points_with_normals_ = points_with_normals_.cols();
  prior_probabilities_ = prior_probabilities;
}

void HybridLineEstimator::LeastSquares(
    const std::vector<std::vector<int>>& sample, Eigen::Vector3d* line) const {
  const int kNumSamplesPoints = static_cast<int>(sample[0].size());
  const int kNumSamplesPointsWithNormals = static_cast<int>(sample[1].size());
  const int kNumSamples = kNumSamplesPoints + kNumSamplesPointsWithNormals;

  if (kNumSamples < 6) return;
  // We fit the line by estimating the eigenvectors of the covariance matrix
  // of the data.
  Eigen::Vector2d mean(0.0, 0.0);
  for (int i = 0; i < kNumSamplesPoints; ++i) {
    mean += points_.col(sample[0][i]);
  }
  for (int i = 0; i < kNumSamplesPointsWithNormals; ++i) {
    mean += points_with_normals_.col(sample[1][i]).head<2>();
  }
  mean /= static_cast<double>(kNumSamples);

  // Builds the covariance matrix C.
  Eigen::Matrix2d C = Eigen::Matrix2d::Zero();

  for (int i = 0; i < kNumSamplesPoints; ++i) {
    Eigen::Vector2d d = points_.col(sample[0][i]) - mean;
    C += d * d.transpose();
  }
  for (int i = 0; i < kNumSamplesPointsWithNormals; ++i) {
    Eigen::Vector2d d = points_with_normals_.col(sample[1][i]).head<2>() - mean;
    C += d * d.transpose();
  }
  C /= static_cast<double>(kNumSamples - 1);

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig_solver(C);
  if (eig_solver.info() != Eigen::Success) return;

  line->head<2>() = eig_solver.eigenvectors().col(1);

  // Re-estimates the translation along the line to account for subtraction
  // of mean.
  (*line)[2] = -line->head<2>().dot(mean);
}

// Evaluates the line on the i-th data point of the t-th data type.
double HybridLineEstimator::EvaluateModelOnPoint(const Eigen::Vector3d& line,
                                                 int t, int i) const {
  double residual = 0.0;
  if (t == 0) {
    residual = line.dot(points_.col(i).homogeneous());
  } else {
    residual = line.dot(points_with_normals_.col(i).head<2>().homogeneous());
  }
  return residual * residual;
}

int HybridLineEstimator::TwoPointSolver(
    const std::vector<int>& sample, std::vector<Eigen::Vector3d>* lines) const {
  lines->clear();
  if (sample.size() < 2u) return 0;

  lines->resize(1);
  Eigen::Vector3d p1(points_(0, sample[0]), points_(1, sample[0]), 1.0);
  Eigen::Vector3d p2(points_(0, sample[1]), points_(1, sample[1]), 1.0);
  (*lines)[0] = p1.cross(p2);
  // Normalizes the line such that the normal of the line has unit length.
  double normal_norm = (*lines)[0].head<2>().norm();
  if (normal_norm == 0.0) {
    lines->clear();
    return 0;
  }

  (*lines)[0] /= normal_norm;

  return 1;
}

int HybridLineEstimator::PointNormalSolver(
    const std::vector<int>& sample, std::vector<Eigen::Vector3d>* lines) const {
  lines->clear();
  if (sample.size() < 1u) return 0;

  lines->resize(1);
  Eigen::Vector2d normal = points_with_normals_.col(sample[0]).tail<2>();
  normal.normalize();
  double c = -normal.dot(points_with_normals_.col(sample[0]).head<2>());
  (*lines)[0] = Eigen::Vector3d(normal[0], normal[1], c);
  // Normalizes the line such that the normal of the line has unit length.
  double normal_norm = (*lines)[0].head<2>().norm();
  if (normal_norm == 0.0) {
    lines->clear();
    return 0;
  }

  (*lines)[0] /= normal_norm;

  return 1;
}

}  // namespace ransac_lib
