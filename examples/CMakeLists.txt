cmake_minimum_required (VERSION 3.0)

if (EXISTS "${CMAKE_SOURCE_DIR}/cmake")
  set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake)
endif ()

set (CMAKE_CXX_STANDARD 11)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

find_package (opengv REQUIRED)

include_directories (
  ${CMAKE_SOURCE_DIR}
)

add_executable (line_estimation line_estimation.cc line_estimator.cc line_estimator.h)
target_link_libraries (line_estimation Eigen3::Eigen)

add_executable (camera_pose_estimation camera_pose_estimation.cc calibrated_absolute_pose_estimator.cc calibrated_absolute_pose_estimator.h)
target_link_libraries (camera_pose_estimation Eigen3::Eigen opengv)