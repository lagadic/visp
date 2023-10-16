cmake_minimum_required (VERSION 3.5)
project(visp)

find_package(VISP REQUIRED)
find_package(pybind11 REQUIRED)
include_directories(include)
include_directories(${VISP_INCLUDE_DIRS})

file(GLOB generated_cpp ${VISP_DIR}/modules/python/bindings/src/*.cpp)
pybind11_add_module(visp THIN_LTO ${generated_cpp})
target_link_libraries(visp PRIVATE ${VISP_LIBRARIES})
