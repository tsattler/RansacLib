cmake_minimum_required (VERSION 3.0)

if (EXISTS "${CMAKE_SOURCE_DIR}/cmake")
  set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake)
endif ()

set (CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package (Eigen3 REQUIRED)

find_package (opengv REQUIRED)

find_package (Ceres REQUIRED)

find_package (Matlab)

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
target_link_libraries (localization opengv ${CERES_LIBRARIES})

add_library (localize STATIC localize.cc calibrated_absolute_pose_estimator.cc calibrated_absolute_pose_estimator.h)
target_link_libraries (localize opengv ${CERES_LIBRARIES})

#add_executable (localization_with_gt localization_with_gt.cc calibrated_absolute_pose_estimator.cc calibrated_absolute_pose_estimator.h)
#target_link_libraries (localization_with_gt opengv ${CERES_LIBRARIES})

add_executable (localization_with_gt_colmap localization_with_gt_colmap.cc calibrated_absolute_pose_estimator.cc calibrated_absolute_pose_estimator.h)
target_link_libraries (localization_with_gt_colmap opengv ${CERES_LIBRARIES})

#add_executable (localization_gc localization_gc.cc #calibrated_absolute_pose_estimator.cc calibrated_absolute_pose_estimator.h)
#target_link_libraries (localization opengv)

IF (Matlab_FOUND)
  matlab_add_mex (NAME CameraPoseRANSAC_Mex SRC CameraPoseRANSAC_Mex.cpp)
  target_link_libraries (CameraPoseRANSAC_Mex localize opengv ${CERES_LIBRARIES} ${MATLAB_LIBRARIES})
  set_target_properties (CameraPoseRANSAC_Mex PROPERTIES COMPILE_FLAGS "-fvisibility=default")
ENDIF()

#This is the "Matlab way" to compile a mex library. However, it crashes when trying to run the localization.
#cd build
#mex -g -I../examples -I/usr/include/eigen3 -L./examples -llocalize -L/usr/local/lib -lopengv -L/usr/local/lib -lceres ../examples/CameraPoseRANSAC_Mex.cpp
