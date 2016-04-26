project(tutorial-robot)

cmake_minimum_required(VERSION 2.6)

find_package(VISP REQUIRED visp_core visp_robot visp_vs visp_gui)

set(tutorial_cpp
  tutorial-pioneer-robot.cpp
  tutorial-simu-pioneer.cpp
  tutorial-simu-pioneer-continuous-gain-constant.cpp
  tutorial-simu-pioneer-continuous-gain-adaptive.cpp
  tutorial-simu-pioneer-pan.cpp)

if(VISP_HAVE_PIONEER)
  # Add specific build flag to turn off warnings coming from libaria 3rd party
  visp_set_source_file_compile_flag(tutorial-pioneer-robot.cpp -Wno-unused-parameter)
endif()

foreach(cpp ${tutorial_cpp})
  visp_add_target(${cpp})
  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "tutorials")
  endif()
endforeach()
