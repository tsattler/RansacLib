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
// python bindings created by NAVER LABS Europe

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <random>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <RansacLib/ransac.h>
#include "calibrated_absolute_pose_estimator.h"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

namespace py = pybind11;

py::dict ransaclib_localization(const std::string query_name,
                                const float focal_x,
                                const float focal_y,
                                const std::vector<Eigen::Vector2d> points2D_vec,
                                const std::vector<Eigen::Vector3d> points3D_vec,
                                const float inlier_threshold,
                                const int number_lo_steps,
                                const uint32_t min_num_iterations,
                                const uint32_t max_num_iterations) {
  using ransac_lib::LocallyOptimizedMSAC;
  using ransac_lib::calibrated_absolute_pose::CalibratedAbsolutePoseEstimator;
  using ransac_lib::calibrated_absolute_pose::CameraPose;
  using ransac_lib::calibrated_absolute_pose::CameraPoses;
  using ransac_lib::calibrated_absolute_pose::Points2D;
  using ransac_lib::calibrated_absolute_pose::Points3D;
  using ransac_lib::calibrated_absolute_pose::ViewingRays;

  std::cout << std::endl << std::endl;

  Points2D points2D(points2D_vec.begin(), points2D_vec.end());
  Points3D points3D(points3D_vec.begin(), points3D_vec.end());

  const int kNumMatches = static_cast<int>(points2D.size());
  
  std::cout << " image " << query_name << " has # " << kNumMatches
            << " matches as input to RANSAC" << std::endl;

  // Failure output dictionary.
  py::dict failure_dict;
  failure_dict["success"] = false;
  if (kNumMatches <= 3) {
    std::cout << " Found only " << kNumMatches << " matches for query image "
              << query_name << " -> skipping image" << std::endl;
    return failure_dict;
  }
  std::cout << "  " << query_name << " "
            << focal_x << " " << focal_y
            << std::endl;
  ViewingRays rays;
  CalibratedAbsolutePoseEstimator::PixelsToViewingRays(
      focal_x, focal_y, points2D, &rays);

  ransac_lib::LORansacOptions options;
  options.min_num_iterations_ = min_num_iterations;
  options.max_num_iterations_ = max_num_iterations;
  options.min_sample_multiplicator_ = 7;
  options.num_lsq_iterations_ = 4;
  options.num_lo_steps_ = number_lo_steps;
  options.lo_starting_iterations_ = 60;
  options.final_least_squares_ = true;

  std::random_device rand_dev;
  options.random_seed_ = rand_dev();

  const double kInThreshPX = static_cast<double>(inlier_threshold);
  options.squared_inlier_threshold_ = kInThreshPX * kInThreshPX;

  CalibratedAbsolutePoseEstimator solver(
      focal_x, focal_y, kInThreshPX * kInThreshPX,
      points2D, rays, points3D);

  LocallyOptimizedMSAC<CameraPose, CameraPoses,
                        CalibratedAbsolutePoseEstimator>
      lomsac;
  ransac_lib::RansacStatistics ransac_stats;
  CameraPose best_model;

  std::cout << "   " << query_name << " : running LO-MSAC on "
            << kNumMatches << " matches " << std::endl;
  auto ransac_start = std::chrono::system_clock::now();

  int num_ransac_inliers =
      lomsac.EstimateModel(options, solver, &best_model, &ransac_stats);
  auto ransac_end = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed_seconds = ransac_end - ransac_start;
  std::cout << "   ... LOMSAC found " << num_ransac_inliers << " inliers in "
            << ransac_stats.num_iterations
            << " iterations with an inlier ratio of "
            << ransac_stats.inlier_ratio << std::endl;
  std::cout << "   ... LOMSAC took " << elapsed_seconds.count() << " s"
            << std::endl;
  std::cout << "   ... LOMSAC executed " << ransac_stats.number_lo_iterations
            << " local optimization stages" << std::endl;

  std::cout << "  Image " << query_name << " : we found # "
            << num_ransac_inliers << " inliers" << std::endl;

  //    if (num_ransac_inliers < 12) continue;

  Eigen::Matrix3d R = best_model.topLeftCorner<3, 3>();
  Eigen::Vector3d t = -R * best_model.col(3);
  Eigen::Quaterniond q(R);
  q.normalize();

  std::cout << query_name << " " << q.w() << " " << q.x() << " " << q.y()
      << " " << q.z() << " " << t[0] << " " << t[1] << " " << t[2]
      << std::endl;

    /// Success output dictionary.
    py::dict success_dict;
    success_dict["success"] = true;
    success_dict["qvec"] = Eigen::Vector4d(q.w(), q.x(), q.y(), q.z());
    success_dict["tvec"] = t;
    success_dict["num_inliers"] = num_ransac_inliers;
    success_dict["inliers"] = ransac_stats.inlier_indices;
    
    return success_dict;
}
