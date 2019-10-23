project(OgreTutorial)

find_package(VISP REQUIRED)
if(VISP_FOUND)
  include(${VISP_USE_FILE})
endif()

add_executable(HelloWorldOgre HelloWorldOgre.cpp)
add_executable(HelloWorldOgreAdvanced HelloWorldOgreAdvanced.cpp)

