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

#include <RansacLib/hybrid_ransac.h>
#include <RansacLib/ransac.h>
#include "hybrid_line_estimator.h"

// Generates a random transformation.
void GenerateRandomTransform(Eigen::Matrix2d* R, Eigen::Vector2d* t) {
  std::random_device rand_dev;
  std::mt19937 rng(rand_dev());
  std::uniform_real_distribution<double> distr(-0.5, 0.5);

  const double kAngle = distr(rng) * M_PI;
  *R << std::cos(kAngle), -std::sin(kAngle), std::sin(kAngle), std::cos(kAngle);
  *t = Eigen::Vector2d(distr(rng) * 10.0, distr(rng) * 10.0);
}

// Assumes that inlier threshold << 0.5.
void GenerateRandomInstance(const int num_inliers, const int num_outliers,
                            double inlier_threshold, const Eigen::Matrix2d& R,
                            const Eigen::Vector2d& t,
                            Eigen::Matrix4Xd* points_with_normals) {
  const int kNumPoints = num_inliers + num_outliers;
  points_with_normals->resize(4, kNumPoints);

  std::vector<int> indices(kNumPoints);
  std::iota(indices.begin(), indices.end(), 0);

  std::random_device rand_dev;
  std::mt19937 rng(rand_dev());

  std::shuffle(indices.begin(), indices.end(), rng);

  // Generates num_inliers points along the x-axis in the interval [0, 1] with
  // a y-value in the range (-inlier_threshold, inlier_threshold) choosen
  // at random. All normals of the inliers are set to [0, 1]
  std::uniform_real_distribution<double> distr(-inlier_threshold,
                                               inlier_threshold);

  const double kXStep = 1.0 / static_cast<double>(num_inliers);
  double x = 0.0;
  for (int i = 0; i < num_inliers; ++i, x += kXStep) {
    const int kIndex = indices[i];
    points_with_normals->col(kIndex)[0] = x;
    while (true) {
      points_with_normals->col(kIndex)[1] = distr(rng);
      if (points_with_normals->col(kIndex)[1] > -inlier_threshold) {
        break;
      }
    }
    points_with_normals->col(kIndex)[2] = 0.0;
    points_with_normals->col(kIndex)[3] = 1.0;
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
    points_with_normals->col(kIndex)[0] = x;
    points_with_normals->col(kIndex)[1] = y;

    points_with_normals->col(kIndex)[2] = 0.0;
    points_with_normals->col(kIndex)[3] = 0.0;
    while (points_with_normals->col(kIndex).tail<2>().norm() < 0.5) {
      points_with_normals->col(kIndex)[2] = distr_y(rng);
      points_with_normals->col(kIndex)[3] = distr_y(rng);
      points_with_normals->col(kIndex).tail<2>().normalize();
    }
  }

  // Rotates and translates the points.
  for (int i = 0; i < kNumPoints; ++i) {
    Eigen::Vector2d p = R * points_with_normals->col(i).head<2>() + t;
    points_with_normals->col(i).head<2>() = p;
    Eigen::Vector2d n = R * points_with_normals->col(i).tail<2>();
    points_with_normals->col(i).tail<2>() = n;
  }
}

int main(int argc, char** argv) {
  ransac_lib::HybridLORansacOptions options;
  options.min_num_iterations_ = 100u;
  options.max_num_iterations_ = 10000u;
  options.max_num_iterations_per_solver_ = 1000u;
  options.squared_inlier_thresholds_ = {0.01 * 0.01, 0.01 * 0.01};
  options.data_type_weights_ = {2.0, 0.5};

  std::random_device rand_dev;
  options.random_seed_ = rand_dev();

  // Generates random instances for outlier ratios 10%, 20%, 30%, ..., 90%,
  // and then applies HybridRANSAC on it.
  const int kNumDataPoints = 100;
  const int kNumDataPointsWithNormals = 100;
  std::vector<double> outlier_ratios = {0.1, 0.2, 0.3, 0.4, 0.5,
                                        0.6, 0.7, 0.8, 0.9};
  for (const double outlier_ratio : outlier_ratios) {
    std::cout << " Inlier ratio: " << 1.0 - outlier_ratio << std::endl;
    int num_outliers_points =
        static_cast<int>(static_cast<double>(kNumDataPoints) * outlier_ratio);
    int num_inliers_points = kNumDataPoints - num_outliers_points;

    Eigen::Matrix2d R;
    Eigen::Vector2d t;
    GenerateRandomTransform(&R, &t);

    Eigen::Matrix4Xd data;
    GenerateRandomInstance(num_inliers_points, num_outliers_points, 0.5 * 0.01,
                           R, t, &data);
    Eigen::Matrix2Xd points(2, data.cols());
    points.row(0) = data.row(0);
    points.row(1) = data.row(1);

    int num_outliers_points_with_normals = static_cast<int>(
        static_cast<double>(kNumDataPointsWithNormals) * outlier_ratio);
    int num_inliers_points_with_normals =
        kNumDataPointsWithNormals - num_outliers_points_with_normals;

    GenerateRandomInstance(num_inliers_points_with_normals,
                           num_outliers_points_with_normals, 0.5 * 0.01, R, t,
                           &data);
    Eigen::Matrix4Xd points_with_normals = data;
    std::cout << "   ... instance generated" << std::endl;

    std::vector<double> prior_probabilities = {0.2, 0.8};
    ransac_lib::HybridLineEstimator solver(points, points_with_normals,
                                           prior_probabilities);
    ransac_lib::HybridLocallyOptimizedMSAC<Eigen::Vector3d,
                                           std::vector<Eigen::Vector3d>,
                                           ransac_lib::HybridLineEstimator>
        lomsac;
    ransac_lib::HybridRansacStatistics ransac_stats;

    std::cout << "   ... running LOMSAC" << std::endl;
    Eigen::Vector3d best_model;
    int num_ransac_inliers =
        lomsac.EstimateModel(options, solver, &best_model, &ransac_stats);
    std::cout << "   ... LOMSAC found " << num_ransac_inliers << " inliers in "
              << ransac_stats.num_iterations_per_solver[0]
              << " iterations of solver 0 and "
              << ransac_stats.num_iterations_per_solver[1]
              << " iterations of solver 1 with inlier ratios of "
              << ransac_stats.inlier_ratios[0] << " and "
              << ransac_stats.inlier_ratios[1] << std::endl;
  }
}
