project(tutorial-visual-servo-ibvs)

cmake_minimum_required(VERSION 2.6)

find_package(VISP REQUIRED visp_core visp_robot visp_vs visp_blob visp_vision visp_io visp_gui)

# set the list of source files
set(tutorial_cpp
  tutorial-ibvs-4pts.cpp
  tutorial-ibvs-4pts-display.cpp
  tutorial-ibvs-4pts-image-tracking.cpp
  tutorial-ibvs-4pts-ogre.cpp
  tutorial-ibvs-4pts-ogre-tracking.cpp
  tutorial-ibvs-4pts-plotter.cpp
  tutorial-ibvs-4pts-plotter-gain-adaptive.cpp
  tutorial-ibvs-4pts-plotter-continuous-gain-adaptive.cpp
  tutorial-ibvs-4pts-wireframe-camera.cpp
  tutorial-ibvs-4pts-wireframe-robot-afma6.cpp
  tutorial-ibvs-4pts-wireframe-robot-viper.cpp
)

list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/sphere/Sphere.mesh" )
list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/sphere/Sphere.material" )
list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/target_square.pgm" )

if(VISP_HAVE_OGRE)
  # Add specific build flag to turn off warnings coming from libogre and libois 3rd party
  visp_set_source_file_compile_flag(tutorial-ibvs-4pts-ogre.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter)
  visp_set_source_file_compile_flag(tutorial-ibvs-4pts-ogre-tracking.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter)
endif()

foreach(cpp ${tutorial_cpp})
  visp_add_target(${cpp})
  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "tutorials")
  endif()
endforeach()

# Copy the data files to the same location than the target
foreach(data ${tutorial_data})
  visp_copy_data(tutorial-ibvs-4pts-ogre.cpp ${data})
endforeach()
