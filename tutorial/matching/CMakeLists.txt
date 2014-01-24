project(tutorial-matching-keypoint)

cmake_minimum_required(VERSION 2.6)

find_package(VISP REQUIRED)

# set the list of source files
set(tutorial_cpp
  tutorial-matching-surf.cpp
  tutorial-matching-surf-homography.cpp)

list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/video-postcard.mpeg" )

foreach(cpp ${tutorial_cpp})
  visp_add_target(${cpp})
endforeach()

# Copy the data files to the same location than the target
foreach(data ${tutorial_data})
  visp_copy_data(tutorial-matching-surf.cpp ${data})
endforeach()
