project(visp-matlab)

cmake_minimum_required(VERSION 2.6)

find_package(VISP REQUIRED visp_core)
include_directories(${VISP_INCLUDE_DIRS})

# Matlab
find_package(Matlab COMPONENTS MX_LIBRARY ENG_LIBRARY REQUIRED)
include_directories(${Matlab_INCLUDE_DIRS})
add_executable(tutorial-matlab tutorial-matlab.cpp)
target_link_libraries(tutorial-matlab ${VISP_LIBRARIES} ${Matlab_LIBRARIES})
