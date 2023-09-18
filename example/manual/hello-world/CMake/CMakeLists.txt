project(HelloWorld)

cmake_minimum_required(VERSION 3.5)

find_package(VISP REQUIRED)
if(VISP_FOUND)
  include(${VISP_USE_FILE})
endif()

add_executable(HelloWorld HelloWorld.cpp)
