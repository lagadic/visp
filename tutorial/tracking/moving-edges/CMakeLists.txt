project(tutorial-tracking-me)

cmake_minimum_required(VERSION 2.6)

find_package(VISP REQUIRED)

# set the list of source files
set(tutorial_cpp
  tutorial-me-ellipse-tracker.cpp
  tutorial-me-line-tracker.cpp)

foreach(cpp ${tutorial_cpp})
  visp_add_target(${cpp})
endforeach()
