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

#include "line_estimator.h"

namespace ransac_lib {

LineEstimator::LineEstimator(const Eigen::Matrix2Xd& data) {
  data_ = data;
  num_data_ = data_.cols();
}

int LineEstimator::MinimalSolver(const std::vector<int>& sample,
                                 std::vector<Eigen::Vector3d>* lines) const {
  lines->clear();
  if (sample.size() < 2u) return 0;

  lines->resize(1);
  Eigen::Vector3d p1(data_(0, sample[0]), data_(1, sample[0]), 1.0);
  Eigen::Vector3d p2(data_(0, sample[1]), data_(1, sample[1]), 1.0);
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

int LineEstimator::NonMinimalSolver(const std::vector<int>& sample,
                                    Eigen::Vector3d* line) const {
  if (sample.size() < 6u) return 0;

  const int kNumSamples = static_cast<int>(sample.size());

  // We fit the line by estimating the eigenvectors of the covariance matrix
  // of the data.
  Eigen::Vector2d mean(0.0, 0.0);
  for (int i = 0; i < kNumSamples; ++i) {
    mean += data_.col(sample[i]);
  }
  mean /= static_cast<double>(kNumSamples);

  // Builds the covariance matrix C.
  Eigen::Matrix2d C = Eigen::Matrix2d::Zero();

  for (int i = 0; i < kNumSamples; ++i) {
    Eigen::Vector2d d = data_.col(sample[i]) - mean;
    C += d * d.transpose();
  }
  C /= static_cast<double>(kNumSamples - 1);

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig_solver(C);
  if (eig_solver.info() != Eigen::Success) return 0;

  line->head<2>() = eig_solver.eigenvectors().col(1);

  // Re-estimates the translation along the line to account for subtraction
  // of mean.
  (*line)[2] = -line->head<2>().dot(mean);

  return 1;
}

// Evaluates the line on the i-th data point.
double LineEstimator::EvaluateModelOnPoint(const Eigen::Vector3d& line,
                                           int i) const {
  double residual = line.dot(data_.col(i).homogeneous());
  return residual * residual;
}

}  // namespace ransac_lib
