cmake_minimum_required (VERSION 3.0)

if (EXISTS "${CMAKE_SOURCE_DIR}/cmake")
  set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake)
endif ()

set (CMAKE_CXX_STANDARD 11)

find_package (Eigen REQUIRED)

find_package (opengv REQUIRED)

include_directories (
  ${CMAKE_SOURCE_DIR}
  ${EIGEN_INCLUDE_DIRS}
)

add_executable (line_estimation line_estimation.cc line_estimator.cc line_estimator.h)

add_executable (camera_pose_estimation camera_pose_estimation.cc calibrated_absolute_pose_estimator.cc calibrated_absolute_pose_estimator.h)
target_link_libraries (camera_pose_estimation opengv)