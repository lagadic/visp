project(tutorial-visual-servo-ibvs)

cmake_minimum_required(VERSION 2.6)

find_package(VISP REQUIRED)

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

foreach(cpp ${tutorial_cpp})
  visp_add_target(${cpp})
endforeach()

# Copy the data files to the same location than the target
foreach(data ${tutorial_data})
  visp_copy_data(tutorial-ibvs-4pts-ogre.cpp ${data})
endforeach()
