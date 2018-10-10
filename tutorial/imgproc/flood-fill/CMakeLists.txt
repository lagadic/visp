project(tutorial-flood-fill)

cmake_minimum_required(VERSION 2.6)

find_package(VISP REQUIRED visp_core visp_io visp_gui visp_imgproc)

# set the list of source files
set(tutorial_cpp
  tutorial-flood-fill.cpp)

foreach(cpp ${tutorial_cpp})
  visp_add_target(${cpp})
  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "tutorials")
  endif()
endforeach()

vp_set_source_file_compile_flag(tutorial-flood-fill.cpp -Wno-strict-overflow)
