project(tutorial-matching-keypoint)

cmake_minimum_required(VERSION 2.6)

find_package(VISP REQUIRED visp_core visp_vision visp_io visp_gui)

# set the list of source files
set(tutorial_cpp
  tutorial-matching-keypoint.cpp
  tutorial-matching-keypoint-SIFT.cpp
  tutorial-matching-keypoint-homography.cpp
  tutorial-matching-surf-deprecated.cpp
  tutorial-matching-surf-homography-deprecated.cpp)

list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/video-postcard.mpeg" )

foreach(cpp ${tutorial_cpp})
  visp_add_target(${cpp})
  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "tutorials")
  endif()
endforeach()

# Copy the data files to the same location than the target
foreach(data ${tutorial_data})
  visp_copy_data(tutorial-matching-keypoint.cpp ${data})
endforeach()
