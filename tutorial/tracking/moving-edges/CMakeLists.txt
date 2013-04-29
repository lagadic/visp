project(tutorial-tracking-me)

cmake_minimum_required(VERSION 2.6)

find_package(VISP REQUIRED)
if(VISP_FOUND)
  include(${VISP_USE_FILE})
endif(VISP_FOUND)

# build the examples
add_executable(tutorial-me-ellipse-tracker tutorial-me-ellipse-tracker.cpp)
add_executable(tutorial-me-line-tracker tutorial-me-line-tracker.cpp)
