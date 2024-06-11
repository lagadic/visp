cmake_minimum_required(VERSION 3.5)

project(ImageManipulation)

find_package(VISP REQUIRED)
if(VISP_FOUND)
  include(${VISP_USE_FILE})
endif()

add_executable(manDisplay        manDisplay.cpp)
add_executable(manGrab1394       manGrab1394.cpp)
add_executable(manGrabDirectShow manGrabDirectShow.cpp)
add_executable(manGrabDisk       manGrabDisk.cpp)
add_executable(manGrabV4l2       manGrabV4l2.cpp)

