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
//     * Neither the name of Torsten Sattler nor the
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

#ifndef RANSACLIB_EXAMPLE_LINE_ESTIMATOR_H_
#define RANSACLIB_EXAMPLE_LINE_ESTIMATOR_H_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <random>
#include <vector>

#include <Eigen/Core>
#include <Eigen/StdVector>

namespace ransac_lib {

// Implements hybrid solver that estimates a line from either two data points
// or from a point with a known normal.
// Solver 1 takes two points as input, solver 2 one point with a normal.
// The first data type is 2D points, the second points with normals.
class HybridLineEstimator {
 public:
  HybridLineEstimator(const Eigen::Matrix2Xd& points,
                      const Eigen::Matrix4Xd& points_with_normals,
                      const std::vector<double>& prior_probabilities);

  inline int num_minimal_solvers() const { return 2; }

  inline void min_sample_sizes(
      std::vector<std::vector<int>>* min_sample_sizes) const {
    min_sample_sizes->resize(2);
    (*min_sample_sizes)[0] = {2, 0};
    (*min_sample_sizes)[1] = {0, 1};
  }

  inline int num_data_types() const { return 2; }

  inline void num_data(std::vector<int>* num_data) const {
    num_data->resize(2);
    (*num_data)[0] = num_points_;
    (*num_data)[1] = num_points_with_normals_;
  }

  inline void solver_probabilities(
      std::vector<double>* solver_probabilites) const {
    *solver_probabilites = prior_probabilities_;
  }

  inline int MinimalSolver(const std::vector<std::vector<int>>& sample,
                           const int solver_idx,
                           std::vector<Eigen::Vector3d>* lines) const {
    if (solver_idx == 0) {
      return TwoPointSolver(sample[0], lines);
    } else {
      return PointNormalSolver(sample[1], lines);
    }
  }

  // Evaluates the line on the i-th data point of the t-th data type.
  double EvaluateModelOnPoint(const Eigen::Vector3d& line, int t, int i) const;

  // Linear least squares solver. Calls NonMinimalSolver.
  void LeastSquares(const std::vector<std::vector<int>>& sample,
                    Eigen::Vector3d* line) const;

 protected:
  int TwoPointSolver(const std::vector<int>& sample,
                     std::vector<Eigen::Vector3d>* lines) const;
  int PointNormalSolver(const std::vector<int>& sample,
                        std::vector<Eigen::Vector3d>* lines) const;

  Eigen::Matrix2Xd points_;
  int num_points_;
  Eigen::Matrix4Xd points_with_normals_;
  int num_points_with_normals_;
  std::vector<double> prior_probabilities_;
};

}  // namespace ransac_lib

#endif  // RANSACLIB_EXAMPLE_LINE_ESTIMATOR_H_
