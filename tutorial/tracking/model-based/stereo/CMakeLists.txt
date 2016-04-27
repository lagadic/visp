project(tutorial-tracking-mb-stereo)

cmake_minimum_required(VERSION 2.6)

find_package(VISP REQUIRED visp_core visp_mbt visp_io visp_gui)

# set the list of source files
set(tutorial_cpp
  tutorial-mb-tracker-stereo.cpp
  tutorial-mb-tracker-stereo-mono.cpp)

list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/teabox.mpg")
list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/teabox.cao")
list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/teabox.init")
list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/teabox.ppm")
list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/teabox.wrl")
list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/teabox.xml")
list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/cRightMcLeft.txt")
list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/teabox_left.init")
list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/teabox_left.mpg")
list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/teabox_left.ppm")
list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/teabox_left.xml")
list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/teabox_right.init")
list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/teabox_right.mpg")
list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/teabox_right.ppm")
list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/teabox_right.xml")

foreach(cpp ${tutorial_cpp})
  visp_add_target(${cpp})

  if(VISP_HAVE_OGRE)
    visp_set_source_file_compile_flag(${cpp} -Wno-unused-parameter -Wno-unused-but-set-parameter)
  endif()

  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "tutorials")
  endif()
endforeach()

# Copy the data files to the same location than the target
foreach(data ${tutorial_data})
  visp_copy_data(tutorial-mb-tracker-stereo-mono.cpp ${data})
endforeach()
