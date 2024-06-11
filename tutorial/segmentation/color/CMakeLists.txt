cmake_minimum_required(VERSION 3.5)

project(color_segmentation)

find_package(VISP REQUIRED visp_core visp_sensor visp_io visp_gui)

# set the list of source files
set(tutorial_cpp
  tutorial-hsv-range-tuner.cpp
  tutorial-hsv-segmentation.cpp
  tutorial-hsv-segmentation-basic.cpp
  tutorial-hsv-segmentation-pcl.cpp
  tutorial-hsv-segmentation-pcl-viewer.cpp
)

file(GLOB tutorial_data "*.jpg")

foreach(cpp ${tutorial_cpp})
  visp_add_target(${cpp})
  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "tutorials")
  endif()
endforeach()

# Copy the data files to the same location than the target
foreach(data ${tutorial_data})
  visp_copy_data(tutorial-hsv-segmentation-basic.cpp ${data})
endforeach()

if(VISP_HAVE_REALSENSE2)
  # Add specific build flag to turn off warnings coming from librealsense 3rd party
  list(APPEND CXX_FLAGS_MUTE_WARNINGS_RS "/wd4244")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS_RS "/wd4267")

  list(APPEND CXX_FLAGS_MUTE_WARNINGS_RS "-Wno-deprecated-copy")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS_RS "-Wno-deprecated-declarations")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS_RS "-Wno-ignored-qualifiers")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS_RS "-Wno-overloaded-virtual")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS_RS "-Wno-pessimizing-move")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS_RS "-Wno-reorder")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS_RS "-Wno-sign-compare")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS_RS "-Wno-unused-function")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS_RS "-Wno-unused-parameter")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS_RS "-Wno-unqualified-std-cast-call")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS_RS "-Wno-missing-field-initializers")
endif()

if(CXX_FLAGS_MUTE_WARNINGS_RS)
  # Add specific build flag to turn off warnings
  visp_set_source_file_compile_flag(tutorial-hsv-range-tuner.cpp ${CXX_FLAGS_MUTE_WARNINGS_RS})
  visp_set_source_file_compile_flag(tutorial-hsv-segmentation.cpp ${CXX_FLAGS_MUTE_WARNINGS_RS})
  visp_set_source_file_compile_flag(tutorial-hsv-segmentation-pcl.cpp ${CXX_FLAGS_MUTE_WARNINGS_RS})
  visp_set_source_file_compile_flag(tutorial-hsv-segmentation-pcl-viewer.cpp ${CXX_FLAGS_MUTE_WARNINGS_RS})
endif()
