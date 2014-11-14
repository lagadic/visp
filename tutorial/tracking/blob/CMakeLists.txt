project(tutorial-tracking-blob)

cmake_minimum_required(VERSION 2.6)

find_package(VISP REQUIRED)

# set the list of source files
set(tutorial_cpp
  tutorial-blob-auto-tracker.cpp
  tutorial-blob-tracker-live-firewire.cpp
  tutorial-blob-tracker-live-v4l2.cpp)

list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/target.pgm" )

foreach(cpp ${tutorial_cpp})
  visp_add_target(${cpp})
  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "tutorials")
  endif()
endforeach()

# Copy the data files to the same location than the target
foreach(data ${tutorial_data})
  visp_copy_data(tutorial-blob-auto-tracker.cpp ${data})
endforeach()
