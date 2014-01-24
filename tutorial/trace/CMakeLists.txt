project(tutorial-trace)

cmake_minimum_required(VERSION 2.6)

find_package(VISP REQUIRED)
include_directories(${VISP_INCLUDE_DIRS})
add_definitions(${VISP_DEFINITIONS})

# set the list of source files
set(tutorial_cpp
  tutorial-trace.cpp)

foreach(cpp ${tutorial_cpp})
  visp_add_target(${cpp})
endforeach()
