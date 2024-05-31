cmake_minimum_required(VERSION 3.5)

project(tutorial-gui-pcl-visualizer)

find_package(VISP REQUIRED visp_core visp_gui visp_io)

# set the list of source files
set(tutorial_cpp
  tutorial-pcl-viewer.cpp
)

foreach(cpp ${tutorial_cpp})
  visp_add_target(${cpp} ClassUsingPclViewer.h ClassUsingPclViewer.cpp)
  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "tutorials")
  endif()
endforeach()

if(VISP_HAVE_PCL)
vp_set_source_file_compile_flag(tutorial-pcl-viewer.cpp -Wno-unused-parameter)
vp_set_source_file_compile_flag(ClassUsingPclViewer.cpp -Wno-unused-parameter)
endif()
