cmake_minimum_required(VERSION 3.5)

project(OgreTutorial)

find_package(VISP REQUIRED)
if(VISP_FOUND)
  include(${VISP_USE_FILE})
endif()

add_executable(HelloWorldOgre HelloWorldOgre.cpp)
add_executable(HelloWorldOgreAdvanced HelloWorldOgreAdvanced.cpp)

if(VISP_HAVE_OGRE)
  # Add specific build flag to turn off warnings coming from libogre and libois 3rd party
  visp_set_source_file_compile_flag(HelloWorldOgre.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter -Wno-overloaded-virtual -Wno-float-equal -Wno-deprecated-copy -Wno-register)
  visp_set_source_file_compile_flag(HelloWorldOgreAdvanced.cpp -Wno-unused-parameter -Wno-unused-but-set-parameter -Wno-overloaded-virtual -Wno-float-equal -Wno-deprecated-copy -Wno-register)
endif()
