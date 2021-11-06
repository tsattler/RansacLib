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

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
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

template <typename T>
double ComputeMedian(std::vector<T>* data) {
  T mean = static_cast<T>(0.0);
  for (size_t i = 0; i < data->size(); ++i) {
    mean += (*data)[i];
  }
  mean /= static_cast<T>(data->size());
  std::cout << " mean : " << mean << std::endl;

  std::sort(data->begin(), data->end());
  if (data->size() % 2u == 1u) {
    return static_cast<double>((*data)[data->size() / 2]);
  } else {
    double a = static_cast<double>((*data)[data->size() / 2 - 1]);
    double b = static_cast<double>((*data)[data->size() / 2]);
    return (a + b) * 0.5;
  }
}

struct QueryData {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::string name;
  double c_x;
  double c_y;

  double focal_x;
  double focal_y;

  int width;
  int height;

  Eigen::Quaterniond q;
  Eigen::Vector3d c;

  std::vector<double> radial;
};

typedef std::vector<QueryData, Eigen::aligned_allocator<QueryData>> Queries;

// Loads the list of query images together with their intrinsics and extrinsics.
bool LoadListIntrinsicsAndExtrinsics(const std::string& filename,
                                     Queries* query_images) {
  std::ifstream ifs(filename.c_str(), std::ios::in);
  if (!ifs.is_open()) {
    std::cerr << " ERROR: Cannot read the image list from " << filename
              << std::endl;
    return false;
  }
  std::string line;

  query_images->clear();

  while (std::getline(ifs, line)) {
    std::stringstream s_stream(line);

    std::string camera_type;

    QueryData q;
    s_stream >> q.name >> camera_type >> q.width >> q.height;
    if (camera_type.compare("SIMPLE_RADIAL") == 0 ||
        camera_type.compare("VSFM") == 0) {
      q.radial.resize(1);
      s_stream >> q.focal_x >> q.c_x >> q.c_y >> q.radial[0];
      q.focal_y = q.focal_x;
    } else if (camera_type.compare("PINHOLE") == 0) {
      q.radial.clear();
      s_stream >> q.focal_x >> q.focal_y >> q.c_x >> q.c_y;
    } else if (camera_type.compare("OPENCV") == 0) {
      // The OPENCV camera model used in Colmap (see
      // https://github.com/colmap/colmap/blob/master/src/base/camera_models.h
      // for details).
      q.radial.resize(4);
      s_stream >> q.focal_x >> q.focal_y >> q.c_x >> q.c_y >> q.radial[0]
               >> q.radial[1] >> q.radial[2] >> q.radial[3];
    } else if (camera_type.compare("VSFM") == 0) {
      q.radial.resize(1);
      s_stream >> q.focal_x >> q.c_x >> q.c_y >> q.radial[0];
      q.focal_y = q.focal_x;
    } else if (camera_type.compare("BROWN_3_PARAMS") == 0) {
      q.radial.resize(3);
      s_stream >> q.focal_x >> q.focal_y >> q.c_x >> q.c_y >> q.radial[0]
               >> q.radial[1] >> q.radial[2];
    }
    s_stream >> q.q.w() >> q.q.x() >> q.q.y() >> q.q.z() >> q.c[0] >> q.c[1] >>
        q.c[2];
    query_images->push_back(q);
  }

  ifs.close();

  return true;
}

// Loads the 2D-3D matches found for that image from a text file.
bool LoadMatches(const std::string& filename, bool invert_Y_Z,
                 ransac_lib::calibrated_absolute_pose::Points2D* points2D,
                 ransac_lib::calibrated_absolute_pose::Points3D* points3D) {
  points2D->clear();
  points3D->clear();

  std::ifstream ifs(filename.c_str(), std::ios::in);
  if (!ifs.is_open()) {
    std::cerr << " ERROR: Cannot read the matches from " << filename
              << std::endl;
    return false;
  }
  std::string line;

  while (std::getline(ifs, line)) {
    std::stringstream s_stream(line);

    Eigen::Vector2d p2D;
    Eigen::Vector3d p3D;
    s_stream >> p2D[0] >> p2D[1] >> p3D[0] >> p3D[1] >> p3D[2];

    if (invert_Y_Z) {
      p3D[1] *= -1.0;
      p3D[2] *= -1.0;
    }

    points2D->push_back(p2D);
    points3D->push_back(p3D);
  }

  return true;
}

int main(int argc, char** argv) {
  using ransac_lib::LocallyOptimizedMSAC;
  using ransac_lib::calibrated_absolute_pose::CalibratedAbsolutePoseEstimator;
  using ransac_lib::calibrated_absolute_pose::CameraPose;
  using ransac_lib::calibrated_absolute_pose::CameraPoses;
  using ransac_lib::calibrated_absolute_pose::Points2D;
  using ransac_lib::calibrated_absolute_pose::Points3D;
  using ransac_lib::calibrated_absolute_pose::ViewingRays;

  std::cout << " usage: " << argv[0] << " images_with_intrinsics outfile "
            << "inlier_threshold num_lo_steps invert_Y_Z points_centered "
            << "[match-file postfix]" << std::endl;
  if (argc < 7) return -1;

  bool invert_Y_Z = static_cast<bool>(atoi(argv[5]));
  bool points_centered = static_cast<bool>(atoi(argv[6]));

  Queries query_data;
  std::string list(argv[1]);

  if (!LoadListIntrinsicsAndExtrinsics(list, &query_data)) {
    std::cerr << " ERROR: Could not read the data from " << list << std::endl;
    return -1;
  }
  const int kNumQuery = static_cast<int>(query_data.size());
  std::cout << " Found " << kNumQuery << " query images " << std::endl;

  std::ofstream ofs(argv[2], std::ios::out);
  if (!ofs.is_open()) {
    std::cerr << " ERROR: Cannot write to " << argv[2] << std::endl;
    return -1;
  }
  
  std::string runtimes_file(argv[2]);
  runtimes_file.append(".times.txt");
  std::ofstream ofs_times(runtimes_file.c_str(), std::ios::out);
  if (!ofs_times.is_open()) {
    std::cerr << " ERROR: Cannot write to " << runtimes_file << std::endl;
    return -1;
  }

  std::string matchfile_postfix = ".individual_datasets.matches.txt";
  if (argc >= 8) {
    matchfile_postfix = std::string(argv[7]);
  }

  std::vector<double> orientation_error(kNumQuery,
                                        std::numeric_limits<double>::max());
  std::vector<double> position_error(kNumQuery,
                                     std::numeric_limits<double>::max());

  std::vector<double> position_thresholds = {0.05, 0.03, 0.02, 0.01};
  std::vector<double> orientation_thresholds = {5.0, 3.0, 2.0, 1.0};
  const int kNumThresholds = 4;
  std::vector<int> num_poses_within_threshold(kNumThresholds, 0);

  double mean_ransac_time = 0.0;

  int num_better_reprojection_error_than_gt = 0;
  int num_reproj_tested = 0;

  for (int i = 0; i < kNumQuery; ++i) {
    std::cout << std::endl << std::endl;

    Points2D points2D;
    Points3D points3D;
    std::string matchfile(query_data[i].name);
    matchfile.append(matchfile_postfix);
    if (!LoadMatches(matchfile, invert_Y_Z, &points2D, &points3D)) {
      std::cerr << "  ERROR: Could not load matches from " << matchfile
                << std::endl;
      continue;
    }
    const int kNumMatches = static_cast<int>(points2D.size());
    if (kNumMatches <= 4) {
      std::cout << " Found only " << kNumMatches << " matches for query image "
                << query_data[i].name << " -> skipping image" << std::endl;

      continue;
    }
    std::cout << "  " << i << " " << query_data[i].name << " "
              << query_data[i].focal_x << " " << query_data[i].focal_y
              << std::endl;

    if (!points_centered) {
      for (int j = 0; j < kNumMatches; ++j) {
        points2D[j][0] -= query_data[i].c_x;
        points2D[j][1] -= query_data[i].c_y;
      }
    }
    ViewingRays rays;
    CalibratedAbsolutePoseEstimator::PixelsToViewingRays(
        query_data[i].focal_x, query_data[i].focal_y, points2D, &rays);

    ransac_lib::LORansacOptions options;
    options.min_num_iterations_ = 100u;
    options.max_num_iterations_ = 10000u;
    options.min_sample_multiplicator_ = 7;
    //    options.num_lsq_iterations_ = 0;
    //    options.num_lo_steps_ = 0;
    options.num_lsq_iterations_ = 4;
    options.num_lo_steps_ = atoi(argv[4]);
    options.lo_starting_iterations_ = 60;
    options.final_least_squares_ = true;
    options.threshold_multiplier_ = 16.0;

    std::random_device rand_dev;
    options.random_seed_ = rand_dev();

    const double kInThreshPX = static_cast<double>(atof(argv[3]));
    options.squared_inlier_threshold_ = kInThreshPX * kInThreshPX;

    CalibratedAbsolutePoseEstimator solver(
        query_data[i].focal_x, query_data[i].focal_y, kInThreshPX * kInThreshPX,
        points2D, rays, points3D);

    LocallyOptimizedMSAC<CameraPose, CameraPoses,
                         CalibratedAbsolutePoseEstimator>
        lomsac;
    ransac_lib::RansacStatistics ransac_stats;
    CameraPose best_model;

    std::cout << "   " << query_data[i].name << " : running LO-MSAC on "
              << kNumMatches << " matches " << std::endl;
    auto ransac_start = std::chrono::system_clock::now();

    int num_ransac_inliers =
        lomsac.EstimateModel(options, solver, &best_model, &ransac_stats);
    auto ransac_end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = ransac_end - ransac_start;
    mean_ransac_time += elapsed_seconds.count();
    std::cout << "   ... LOMSAC found " << num_ransac_inliers << " inliers in "
              << ransac_stats.num_iterations
              << " iterations with an inlier ratio of "
              << ransac_stats.inlier_ratio << std::endl;
    std::cout << "   ... LOMSAC took " << elapsed_seconds.count() << " s"
              << std::endl;
    std::cout << "   ... LOMSAC executed " << ransac_stats.number_lo_iterations
              << " local optimization stages" << std::endl;

    ofs_times << elapsed_seconds.count() << std::endl;
    //     if (num_ransac_inliers < 12) continue;
    if (num_ransac_inliers < 4) continue;

    Eigen::Matrix3d R = best_model.topLeftCorner<3, 3>();
    Eigen::Vector3d t = -R * best_model.col(3);
    Eigen::Quaterniond q(R);
    q.normalize();

    // Measures the pose error.
    double c_error = (best_model.col(3) - query_data[i].c).norm();
    Eigen::Matrix3d R1 = R.transpose();
    Eigen::Matrix3d R2(query_data[i].q);
    Eigen::AngleAxisd aax(R1 * R2);
    double q_error = aax.angle() * 180.0 / M_PI;
    orientation_error[i] = q_error;
    position_error[i] = c_error;
    std::cout << "   pose error: " << c_error << ", " << q_error << std::endl;
    std::cout << best_model.col(3).transpose() << std::endl;
    std::cout << q.coeffs().transpose() << std::endl;
    std::cout << query_data[i].q.coeffs().transpose() <<std::endl;

    for (int k = 0; k < kNumThresholds; ++k) {
      if (c_error <= position_thresholds[k] &&
          q_error <= orientation_thresholds[k]) {
        num_poses_within_threshold[k] += 1;
      }
    }

    ofs << query_data[i].name << " " << q.w() << " " << q.x() << " " << q.y()
        << " " << q.z() << " " << t[0] << " " << t[1] << " " << t[2]
        << std::endl;
  }

  std::sort(orientation_error.begin(), orientation_error.end());
  std::sort(position_error.begin(), position_error.end());

  std::string statistics_file(argv[2]);
  statistics_file.append(".stats.txt");
  std::ofstream ofs_stats(statistics_file.c_str(), std::ios::out);
  if (!ofs_stats.is_open()) {
    std::cerr << " ERROR: Cannot write to " << statistics_file << std::endl;
    return -1;
  }
  
  std::cout << std::endl
            << " Mean RANSAC time: "
            << mean_ransac_time / static_cast<double>(kNumQuery) << " s "
            << std::endl;
  ofs_stats << std::endl
            << " Mean RANSAC time: "
            << mean_ransac_time / static_cast<double>(kNumQuery) << " s "
            << std::endl;

  
  
  double median_pos = ComputeMedian<double>(&position_error);
  double median_rot = ComputeMedian<double>(&orientation_error);
  std::cout << " Median position error: " << median_pos << "m "
            << " Median orientation error: " << median_rot << " deg"
            << std::endl;
  ofs_stats << " Median position error: " << median_pos << "m "
            << " Median orientation error: " << median_rot << " deg"
            << std::endl;
  
  for (int k = 0; k < kNumThresholds; ++k) {
    std::cout << " % images within " << position_thresholds[k] * 100.0
              << "cm and " << orientation_thresholds[k] << "deg: "
              << static_cast<double>(num_poses_within_threshold[k]) /
                     static_cast<double>(kNumQuery) * 100.0
              << std::endl;
    ofs_stats << " % images within " << position_thresholds[k] * 100.0
              << "cm and " << orientation_thresholds[k] << "deg: "
              << static_cast<double>(num_poses_within_threshold[k]) /
                     static_cast<double>(kNumQuery) * 100.0
              << std::endl;
  }
  ofs.close();
  ofs_stats.close();
  ofs_times.close();
  return 0;
}
