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
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <limits>
#include <random>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>


#include <RansacLib/ransac.h>
#include "calibrated_absolute_pose_estimator.h"

namespace ransac_lib {
namespace calibrated_absolute_pose {

void GenerateRandomInstance(const double width, const double height,
                            const double focal_length, const int num_inliers,
                            const int num_outliers, double inlier_threshold,
                            const double min_depth, const double max_depth,
                            Points2D* points2D, ViewingRays* rays,
                            Points3D* points3D) {
  const int kNumPoints = num_inliers + num_outliers;
  points2D->resize(kNumPoints);
  points3D->resize(kNumPoints);

  std::vector<int> indices(kNumPoints);
  std::iota(indices.begin(), indices.end(), 0);

  std::random_device rand_dev;
  std::mt19937 rng(rand_dev());

  std::shuffle(indices.begin(), indices.end(), rng);

  const double kWidthHalf = width * 0.5;
  const double kHeightHalf = height * 0.5;
  std::uniform_real_distribution<double> distr_x(-kWidthHalf, kWidthHalf);
  std::uniform_real_distribution<double> distr_y(-kHeightHalf, kHeightHalf);
  std::uniform_real_distribution<double> distr_d(min_depth, max_depth);
  std::uniform_real_distribution<double> distr(-1.0, 1.0);

  // Generates the inliers.
  for (int i = 0; i < num_inliers; ++i) {
    const int kIndex = indices[i];
    (*points2D)[kIndex] = Eigen::Vector2d(distr_x(rng), distr_y(rng));

    Eigen::Vector3d dir = (*points2D)[kIndex].homogeneous();
    dir.head<2>() /= focal_length;
    dir.normalize();

    // Obtains the 3D point.
    (*points3D)[kIndex] = dir * distr_d(rng);

    // Adds some noise to the 2D position to make the case more realistic.
    (*points2D)[kIndex] +=
        Eigen::Vector2d(distr(rng), distr(rng)) * inlier_threshold;
  }

  // Generates the outlier.
  for (int i = num_inliers; i < kNumPoints; ++i) {
    const int kIndex = indices[i];
    Eigen::Vector2d p(distr_x(rng), distr_y(rng));

    Eigen::Vector3d dir = p.homogeneous();
    dir.head<2>() /= focal_length;
    dir.normalize();

    // Obtains the 3D point.
    (*points3D)[kIndex] = dir * distr_d(rng);

    // Estimates a new pixel position that is far enough from the original one.
    (*points2D)[kIndex] = Eigen::Vector2d(distr_x(rng), distr_y(rng));
    while (((*points2D)[kIndex] - p).norm() < 10.0 * (inlier_threshold + 1.0)) {
      (*points2D)[kIndex] = Eigen::Vector2d(distr_x(rng), distr_y(rng));
    }
  }

  // Randomly rotates and translates the 3D points.
  Eigen::Quaterniond q = Eigen::Quaterniond::UnitRandom();
  Eigen::Matrix3d R(q);
  std::uniform_real_distribution<double> distr_scale(1.0, 2.0);
  Eigen::Vector3d t(distr(rng), distr(rng), distr(rng));
  t *= distr_scale(rng);

  for (int i = 0; i < kNumPoints; ++i) {
    Eigen::Vector3d p = R * (*points3D)[i] + t;
    (*points3D)[i] = p;
  }

  CalibratedAbsolutePoseEstimator::PixelsToViewingRays(
      focal_length, focal_length, *points2D, rays);
}

}  // namespace calibrated_absolute_pose
}  // namespace ransac_lib

int main(int argc, char** argv) {
  ransac_lib::LORansacOptions options;
  options.min_num_iterations_ = 100u;
  options.max_num_iterations_ = 1000000u;

  std::random_device rand_dev;
  options.random_seed_ = rand_dev();

  using ransac_lib::calibrated_absolute_pose::Points3D;
  using ransac_lib::calibrated_absolute_pose::ViewingRays;

  // Generates random instances for outlier ratios 10%, 20%, 30%, ..., 90%,
  // and then applies RANSAC on it.
  // kNumDataPoints data points are used.
  const int kNumDataPoints = 2000;

  std::vector<double> outlier_ratios = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5,
                                        0.6, 0.7, 0.8, 0.9, 0.95};

  const double kWidth = 640.0;
  const double kHeight = 320.0;
  const double kFocalLength = (kWidth * 0.5) / std::tan(60.0 * M_PI / 180.0);
  const double kInThreshPX = 12.0;

  options.squared_inlier_threshold_ = kInThreshPX * kInThreshPX;

  for (const double outlier_ratio : outlier_ratios) {
    std::cout << " Inlier ratio: " << 1.0 - outlier_ratio << std::endl;
    int num_outliers =
        static_cast<int>(static_cast<double>(kNumDataPoints) * outlier_ratio);
    int num_inliers = kNumDataPoints - num_outliers;

    ransac_lib::calibrated_absolute_pose::Points2D points2D;
    ViewingRays rays;
    Points3D points3D;
    ransac_lib::calibrated_absolute_pose::GenerateRandomInstance(
        kWidth, kHeight, kFocalLength, num_inliers, num_outliers, 2.0, 2.0,
        10.0, &points2D, &rays, &points3D);
    std::cout << "   ... instance generated" << std::endl;

    ransac_lib::calibrated_absolute_pose::CalibratedAbsolutePoseEstimator
        solver(kFocalLength, kFocalLength, kInThreshPX * kInThreshPX, points2D,
               rays, points3D);

    // Runs LO-MSAC, as described in Lebeda et al., BMVC 2012.
    std::cout << "   ... running LO-MSAC" << std::endl;
    {
      options.min_sample_multiplicator_ = 7;
      options.num_lsq_iterations_ = 4;
      options.num_lo_steps_ = 10;

      ransac_lib::LocallyOptimizedMSAC<
          ransac_lib::calibrated_absolute_pose::CameraPose,
          ransac_lib::calibrated_absolute_pose::CameraPoses,
          ransac_lib::calibrated_absolute_pose::CalibratedAbsolutePoseEstimator>
          lomsac;
      ransac_lib::RansacStatistics ransac_stats;
      auto ransac_start = std::chrono::system_clock::now();
      ransac_lib::calibrated_absolute_pose::CameraPose best_model;
      int num_ransac_inliers =
          lomsac.EstimateModel(options, solver, &best_model, &ransac_stats);
      auto ransac_end = std::chrono::system_clock::now();
      std::chrono::duration<double> elapsed_seconds = ransac_end - ransac_start;
      std::cout << "   ... LOMSAC found " << num_ransac_inliers
                << " inliers in " << ransac_stats.num_iterations
                << " iterations with an inlier ratio of "
                << ransac_stats.inlier_ratio << std::endl;
      std::cout << "   ... LOMSAC took " << elapsed_seconds.count() << " s"
                << std::endl;
    }
  }
  return 0;
}
