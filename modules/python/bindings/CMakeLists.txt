cmake_minimum_required (VERSION 3.5)
project(visp)

find_package(pybind11 REQUIRED)
find_package(VISP REQUIRED)
include_directories(${VISP_INCLUDE_DIRS})
include_directories(include)

file(GLOB generated_cpp src/*.cpp)
file(GLOB static_cpp src/static/*.cpp)
pybind11_add_module(visp THIN_LTO ${generated_cpp} ${static_cpp})
target_link_libraries(visp PRIVATE ${VISP_LIBRARIES})
