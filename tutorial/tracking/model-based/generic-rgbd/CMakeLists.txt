project(tutorial-tracking-mb-generic-rgbd)

cmake_minimum_required(VERSION 2.6)

find_package(VISP REQUIRED visp_core visp_mbt visp_io visp_gui visp_sensor)

# set the list of source files
set(tutorial_cpp
  tutorial-mb-generic-tracker-rgbd.cpp
  tutorial-mb-generic-tracker-rgbd-realsense.cpp)

file(GLOB tutorial_data data/color_image_*.jpg data/depth_image_*.bin)
list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/cube_depth.xml")
list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/cube.xml")
list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/cube.cao")
list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/cube.init")
list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/cube.ppm")
list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/depth_M_color.txt")

foreach(cpp ${tutorial_cpp})
  visp_add_target(${cpp})

  if(VISP_HAVE_OGRE)
    # Add specific build flag to turn off warnings coming from Ogre3D 3rd party
    list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-ignored-qualifiers")
    list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-unused-parameter")
    list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-unused-but-set-parameter")
    list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-overloaded-virtual")
  endif()
  if(VISP_HAVE_REALSENSE2)
    # Add specific build flag to turn off warnings coming from librealsense 3rd party
    list(APPEND CXX_FLAGS_MUTE_WARNINGS "/wd4244")
    list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-unused-function")
    list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-sign-compare")
    list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-reorder")
    list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-overloaded-virtual")
  endif()

  if(VISP_HAVE_PCL)
    list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-missing-field-initializers")
  endif()

  if(CXX_FLAGS_MUTE_WARNINGS)
    # Add specific build flag to turn off warnings
    visp_set_source_file_compile_flag(${cpp} ${CXX_FLAGS_MUTE_WARNINGS})
  endif()

  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "tutorials")
  endif()
endforeach()

# Copy the data files to the same location than the target
foreach(data ${tutorial_data})
  visp_copy_data(tutorial-mb-generic-tracker-rgbd.cpp ${data})
endforeach()
