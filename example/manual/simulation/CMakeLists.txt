cmake_minimum_required(VERSION 3.5)

project(ImageManipulation)

find_package(VISP REQUIRED)
if(VISP_FOUND)
  include(${VISP_USE_FILE})
endif()

add_executable(manServo4PointsDisplay manServo4PointsDisplay.cpp)
add_executable(manSimu4Dots           manSimu4Dots.cpp)
add_executable(manSimu4Points         manSimu4Points.cpp)

