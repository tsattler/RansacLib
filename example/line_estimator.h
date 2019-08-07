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

// Implements a simple solver that estimates a line from two data points.
class LineEstimator {
 public:
  LineEstimator(const Eigen::Matrix2Xd& data);

  inline int min_sample_size() const { return 2; }

  inline int non_minimal_sample_size() const { return 6; }

  inline int num_data() const { return num_data_; }

  int MinimalSolver(const std::vector<int>& sample,
                    std::vector<Eigen::Vector3d>* lines) const;

  // Returns 0 if no model could be estimated and 1 otherwise.
  // Implemented by a simple linear least squares solver.
  int NonMinimalSolver(const std::vector<int>& sample,
                       Eigen::Vector3d* line) const;

  // Evaluates the line on the i-th data point.
  double EvaluateModelOnPoint(const Eigen::Vector3d& line, int i) const;

  // Linear least squares solver. Calls NonMinimalSolver.
  inline void LeastSquares(const std::vector<int>& sample,
                           Eigen::Vector3d* line) const {
    NonMinimalSolver(sample, line);
  }

 protected:
  // Matrix holding the 2D points through which the line is fitted.
  Eigen::Matrix2Xd data_;
  int num_data_;
};

}  // namespace ransac_lib

#endif  // RANSACLIB_EXAMPLE_LINE_ESTIMATOR_H_
