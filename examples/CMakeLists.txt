cmake_minimum_required (VERSION 3.0)

if (EXISTS "${CMAKE_SOURCE_DIR}/cmake")
  set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake)
endif ()

#SET(CMAKE_CXX_FLAGS "-std=gnu++17")

find_package (Eigen3 REQUIRED)

find_package (opengv REQUIRED)

find_package (Ceres REQUIRED)

add_definitions (-march=native)

include_directories (
  ${CMAKE_SOURCE_DIR}
  ${EIGEN_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

add_executable (line_estimation line_estimation.cc line_estimator.cc line_estimator.h)

add_executable (hybrid_line_estimation hybrid_line_estimation.cc hybrid_line_estimator.cc hybrid_line_estimator.h)

add_executable (camera_pose_estimation camera_pose_estimation.cc calibrated_absolute_pose_estimator.cc calibrated_absolute_pose_estimator.h)
target_link_libraries (camera_pose_estimation opengv ${CERES_LIBRARIES})

add_executable (localization localization.cc calibrated_absolute_pose_estimator.cc calibrated_absolute_pose_estimator.h)
target_link_libraries (localization opengv
                                    ${CERES_LIBRARIES})

add_executable (localization_with_gt localization_with_gt.cc calibrated_absolute_pose_estimator.cc calibrated_absolute_pose_estimator.h)
target_link_libraries (localization_with_gt opengv ${CERES_LIBRARIES})

#add_executable (localization_gc localization_gc.cc #calibrated_absolute_pose_estimator.cc calibrated_absolute_pose_estimator.h)
#target_link_libraries (localization opengv)
