project(tutorial-tracking-mb-generic-rgbd-blender)

cmake_minimum_required(VERSION 2.8.12)

find_package(VISP REQUIRED visp_core visp_mbt visp_io visp_gui)

# set the list of source files
set(tutorial_cpp
  tutorial-mb-generic-tracker-rgbd-blender.cpp)

list(APPEND tutorial_data_dir "camera_poses")
list(APPEND tutorial_data_dir "depth")
list(APPEND tutorial_data_dir "images")

list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/depth_M_color.txt")
list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/teabox.cao")
list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/teabox.init")
list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/teabox.ppm")
list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/teabox.xml")
list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/teabox_depth.xml")

foreach(cpp ${tutorial_cpp})
  visp_add_target(${cpp})

  if(VISP_HAVE_OGRE)
    # Add specific build flag to turn off warnings coming from Ogre3D 3rd party
    list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-ignored-qualifiers")
    list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-unused-parameter")
    list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-unused-but-set-parameter")
    list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-overloaded-virtual")
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
foreach(data_dir ${tutorial_data_dir})
  visp_copy_dir(tutorial-mb-generic-tracker-rgbd-blender.cpp "${CMAKE_CURRENT_SOURCE_DIR}" ${data_dir})
endforeach()

foreach(data ${tutorial_data})
  visp_copy_data(tutorial-mb-generic-tracker-rgbd-blender.cpp ${data})
endforeach()
