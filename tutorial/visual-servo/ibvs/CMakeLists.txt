project(tutorial-visual-servo)

cmake_minimum_required(VERSION 2.6)

find_package(VISP REQUIRED)
if(VISP_FOUND)
  include(${VISP_USE_FILE})
endif(VISP_FOUND)

# build the examples
add_executable(tutorial-ibvs-4pts tutorial-ibvs-4pts.cpp)
add_executable(tutorial-ibvs-4pts-display tutorial-ibvs-4pts-display.cpp)
add_executable(tutorial-ibvs-4pts-image-tracking tutorial-ibvs-4pts-image-tracking.cpp)
add_executable(tutorial-ibvs-4pts-ogre tutorial-ibvs-4pts-ogre.cpp)
add_executable(tutorial-ibvs-4pts-ogre-tracking tutorial-ibvs-4pts-ogre-tracking.cpp)
add_executable(tutorial-ibvs-4pts-plotter tutorial-ibvs-4pts-plotter.cpp)
add_executable(tutorial-ibvs-4pts-wireframe-camera tutorial-ibvs-4pts-wireframe-camera.cpp)
add_executable(tutorial-ibvs-4pts-wireframe-robot-afma6 tutorial-ibvs-4pts-wireframe-robot-afma6.cpp)
add_executable(tutorial-ibvs-4pts-wireframe-robot-viper tutorial-ibvs-4pts-wireframe-robot-viper.cpp)

# copy the Ogre3D data
get_target_property(target_location tutorial-ibvs-4pts-ogre LOCATION)
get_filename_component(target_location "${target_location}" PATH)
list(APPEND data2copy "${CMAKE_CURRENT_SOURCE_DIR}/sphere/Sphere.mesh" )
list(APPEND data2copy "${CMAKE_CURRENT_SOURCE_DIR}/sphere/Sphere.material" )
list(APPEND data2copy "${CMAKE_CURRENT_SOURCE_DIR}/target_square.pgm" )
foreach(data ${data2copy})
  add_custom_command(
    TARGET tutorial-ibvs-4pts-ogre
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy "${data}" "${target_location}"
  )
endforeach()
