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
#include <iostream>
#include <limits>
#include <random>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include <RansacLib/ransac.h>
#include "line_estimator.h"

// Assumes that inlier threshold << 0.5.
void GenerateRandomInstance(const int num_inliers, const int num_outliers,
                            double inlier_threshold, Eigen::Matrix2Xd* points) {
  const int kNumPoints = num_inliers + num_outliers;
  points->resize(2, kNumPoints);

  std::vector<int> indices(kNumPoints);
  std::iota(indices.begin(), indices.end(), 0);

  std::random_device rand_dev;
  std::mt19937 rng(rand_dev());

  std::shuffle(indices.begin(), indices.end(), rng);

  // Generates num_inliers points along the x-axis in the interval [0, 1] with
  // a y-value in the range (-inlier_threshold, inlier_threshold) choosen
  // at random.
  std::uniform_real_distribution<double> distr(-inlier_threshold,
                                               inlier_threshold);

  const double kXStep = 1.0 / static_cast<double>(num_inliers);
  double x = 0.0;
  for (int i = 0; i < num_inliers; ++i, x += kXStep) {
    const int kIndex = indices[i];
    points->col(kIndex)[0] = x;
    while (true) {
      points->col(kIndex)[1] = distr(rng);
      if (points->col(kIndex)[1] > -inlier_threshold) {
        break;
      }
    }
  }

  // Randomly generates outliers in the range [0, 1] x [-0.5, 0.5].
  std::uniform_real_distribution<double> distr_x(0.0, 1.0);
  std::uniform_real_distribution<double> distr_y(-0.5, 0.5);
  for (int i = num_inliers; i < kNumPoints; ++i) {
    double x = distr_x(rng);
    double y = distr_y(rng);
    while (std::fabs(y) < 5.0 * inlier_threshold) {
      y = distr_y(rng);
    }

    const int kIndex = indices[i];
    points->col(kIndex)[0] = x;
    points->col(kIndex)[1] = y;
  }

  // Randomly rotates and translates the points.
  const double kAngle = distr_y(rng) * M_PI;
  Eigen::Matrix2d R;
  R << std::cos(kAngle), -std::sin(kAngle), std::sin(kAngle), std::cos(kAngle);
  Eigen::Vector2d t(distr_y(rng) * 10.0, distr_y(rng) * 10.0);

  for (int i = 0; i < kNumPoints; ++i) {
    Eigen::Vector2d p = R * points->col(i) + t;
    points->col(i) = p;
  }
}

int main(int argc, char** argv) {
  ransac_lib::LORansacOptions options;
  options.min_num_iterations_ = 100u;
  options.max_num_iterations_ = 100000u;
  options.squared_inlier_threshold_ = 0.01 * 0.01;

  std::random_device rand_dev;
  options.random_seed_ = rand_dev();

  // Generates random instances for outlier ratios 10%, 20%, 30%, ..., 90%,
  // and then applies RANSAC on it.
  // kNumDataPoints data points are used.
  const int kNumDataPoints = 10000;
  std::vector<double> outlier_ratios = {0.1, 0.2, 0.3, 0.4,  0.5,  0.6,
                                        0.7, 0.8, 0.9, 0.95, 0.99, 0.999};
  for (const double outlier_ratio : outlier_ratios) {
    std::cout << " Inlier ratio: " << 1.0 - outlier_ratio << std::endl;
    int num_outliers =
        static_cast<int>(static_cast<double>(kNumDataPoints) * outlier_ratio);
    int num_inliers = kNumDataPoints - num_outliers;

    Eigen::Matrix2Xd data;
    GenerateRandomInstance(num_inliers, num_outliers, 0.5 * 0.01, &data);
    std::cout << "   ... instance generated" << std::endl;

    ransac_lib::LineEstimator solver(data);
    ransac_lib::LocallyOptimizedMSAC<Eigen::Vector3d,
                                     std::vector<Eigen::Vector3d>,
                                     ransac_lib::LineEstimator>
        lomsac;
    ransac_lib::RansacStatistics ransac_stats;

    std::cout << "   ... running LOMSAC" << std::endl;
    Eigen::Vector3d best_model;
    int num_ransac_inliers =
        lomsac.EstimateModel(options, solver, &best_model, &ransac_stats);
    std::cout << "   ... LOMSAC found " << num_ransac_inliers << " inliers in "
              << ransac_stats.num_iterations << " iterations with an inlier "
              << "ratio of " << ransac_stats.inlier_ratio << std::endl;
  }
}
