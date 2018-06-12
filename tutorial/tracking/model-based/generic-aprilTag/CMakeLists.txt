project(tutorial-tracking-mb-aprilTag-init)

cmake_minimum_required(VERSION 2.6)

find_package(VISP REQUIRED visp_core visp_sensor visp_gui visp_vision visp_mbt visp_detection)

# set the list of source files
set(tutorial_cpp
  tutorial-mb-generic-tracker-aprilTag-init.cpp
)

foreach(cpp ${tutorial_cpp})
  visp_add_target(${cpp})

  if(VISP_HAVE_OGRE)
    visp_set_source_file_compile_flag(${cpp} -Wno-unused-parameter -Wno-unused-but-set-parameter -Wno-overloaded-virtual)
  endif()

  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "tutorials")
  endif()
endforeach()
