# - Try to find the PoseLib libray
# Once done this will define
#  POSELIB_FOUND       - system has the PoseLib library
#  POSELIB_INCLUDE_DIR - the PoseLib include directory
#  POSELIB_LIBRARY     - Link these to use PoseLib
#  POSELIB_LIBRARY_DIR - Library DIR of PoseLib

if (POSELIB_INCLUDE_DIR)
  # in cache already
  set(POSELIB_FIND_QUIETLY TRUE)
else (POSELIB_INCLUDE_DIR)
  find_path(POSELIB_INCLUDE_DIR PoseLib/p3p.h
      PATHS
      "/usr/local/include"
    )

  if(POSELIB_INCLUDE_DIR)
    set(POSELIB_FOUND TRUE)
    set(POSELIB_LIBRARY_DIR "/usr/local/lib" )

  else(POSELIB_INCLUDE_DIR)
    set(POSELIB_FOUND FALSE)
  endif(POSELIB_INCLUDE_DIR)
endif(POSELIB_INCLUDE_DIR)

find_library(POSELIB_LIBRARY NAMES poselib HINTS "/Users/sattlert/code/PoseLib/_install" "/home/torsat/code/external/PoseLib/_install")
