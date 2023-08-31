project(tutorial-hough)

cmake_minimum_required(VERSION 3.0)

find_package(VISP REQUIRED visp_core visp_gui visp_imgproc)

# set the list of source files
set(tutorial_cpp
  tutorial-circle-hough.cpp
  )

# list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/config")
list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/coins2.jpg")

foreach(cpp ${tutorial_cpp})
  visp_add_target(${cpp} drawingHelpers.cpp)
  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "tutorials")
  endif()
endforeach()

# Copy the data files to the same location than the target
visp_copy_dir(tutorial-circle-hough "${CMAKE_CURRENT_SOURCE_DIR}" config)
foreach(data ${tutorial_data})
  visp_copy_data(tutorial-circle-hough.cpp ${data})
endforeach()
