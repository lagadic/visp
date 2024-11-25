cmake_minimum_required(VERSION 3.10)

project(man-moment)

find_package(VISP REQUIRED)
if(VISP_FOUND)
  include(${VISP_USE_FILE})
endif()

add_executable(manServoMomentsSimple manServoMomentsSimple.cpp)
