cmake_minimum_required (VERSION 3.0)

if (EXISTS "${CMAKE_SOURCE_DIR}/cmake")
  set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake)
endif ()

SET(CMAKE_CXX_FLAGS "-std=gnu++11")

find_package (Eigen3 REQUIRED)

find_package (Ceres REQUIRED)

find_package (PoseLib REQUIRED)

add_definitions (-march=native)
if(MSVC)	
  add_definitions(-D_USE_MATH_DEFINES)
endif()

include_directories (
  ${CMAKE_SOURCE_DIR}
  ${EIGEN3_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${POSELIB_INCLUDE_DIRS}
)

add_executable (line_estimation line_estimation.cc line_estimator.cc line_estimator.h)

add_executable (hybrid_line_estimation hybrid_line_estimation.cc hybrid_line_estimator.cc hybrid_line_estimator.h)

add_executable (camera_pose_estimation camera_pose_estimation.cc calibrated_absolute_pose_estimator.cc calibrated_absolute_pose_estimator.h)
target_link_libraries (camera_pose_estimation ${POSELIB_LIBRARIES} ${CERES_LIBRARIES})

add_executable (localization localization.cc calibrated_absolute_pose_estimator.cc calibrated_absolute_pose_estimator.h)
target_link_libraries (localization ${POSELIB_LIBRARIES} ${CERES_LIBRARIES})

add_executable (localization_with_gt localization_with_gt.cc calibrated_absolute_pose_estimator.cc calibrated_absolute_pose_estimator.h)
target_link_libraries (localization_with_gt ${POSELIB_LIBRARIES} ${CERES_LIBRARIES})

#add_executable (localization_with_gt_colmap localization_with_gt_colmap.cc calibrated_absolute_pose_estimator.cc calibrated_absolute_pose_estimator.h)
#target_link_libraries (localization_with_gt_colmap ${POSELIB_LIBRARIES} ${CERES_LIBRARIES})

#add_executable (localization_gc localization_gc.cc #calibrated_absolute_pose_estimator.cc calibrated_absolute_pose_estimator.h)
#target_link_libraries (localization ${POSELIB_LIBRARIES})

pybind11_add_module(pyransaclib pyransaclib.cc calibrated_absolute_pose_estimator.cc calibrated_absolute_pose_estimator.h)
target_link_libraries(pyransaclib PRIVATE ${POSELIB_LIBRARIES} ${CERES_LIBRARIES})