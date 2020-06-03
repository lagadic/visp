project(visp-mbot)

cmake_minimum_required(VERSION 2.6)

# By default set release configuration
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif()

find_package(VISP REQUIRED visp_core visp_detection visp_io visp_gui visp_sensor visp_vs)

set(tutorial_cpp
  mbot-apriltag-ibvs.cpp
  mbot-apriltag-pbvs.cpp
  mbot-apriltag-2D-half-vs.cpp
  test-serial-mbot.cpp
)

foreach(cpp ${tutorial_cpp})
  visp_add_target(${cpp})
  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "tutorials")
  endif()
endforeach()

