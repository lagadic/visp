project(tutorial-robot)

cmake_minimum_required(VERSION 2.6)

if(CMAKE_SYSTEM_NAME MATCHES WindowsPhone OR CMAKE_SYSTEM_NAME MATCHES WindowsStore)
  set(WINRT TRUE)
endif(CMAKE_SYSTEM_NAME MATCHES WindowsPhone OR CMAKE_SYSTEM_NAME MATCHES WindowsStore)

if(WINRT)
  add_definitions(-DWINRT)
  
  if(CMAKE_SYSTEM_NAME MATCHES WindowsPhone)
    set(WINRT_PHONE TRUE)
    add_definitions(-DWINRT_PHONE)
  elseif(CMAKE_SYSTEM_NAME MATCHES WindowsStore)
    set(WINRT_STORE TRUE)
    add_definitions(-DWINRT_STORE)
  endif()

  if(CMAKE_SYSTEM_VERSION MATCHES 10 OR CMAKE_SYSTEM_VERSION MATCHES 10.0)
    set(WINRT_10 TRUE)
    add_definitions(-DWINRT_10)
  elseif(CMAKE_SYSTEM_VERSION MATCHES 8.1)
    set(WINRT_8_1 TRUE)
    add_definitions(-DWINRT_8_1)
  elseif(CMAKE_SYSTEM_VERSION MATCHES 8.0)
    set(WINRT_8_0 TRUE)
    add_definitions(-DWINRT_8_0)
  endif()
endif()

find_package(VISP REQUIRED visp_core visp_robot visp_vs visp_gui)

set(tutorial_cpp
  tutorial-pioneer-robot.cpp
  tutorial-simu-pioneer.cpp
  tutorial-simu-pioneer-pan.cpp
)

if(NOT DEFINED WINRT_8_1)
  list(APPEND tutorial_cpp tutorial-simu-pioneer-continuous-gain-constant.cpp)
  list(APPEND tutorial_cpp tutorial-simu-pioneer-continuous-gain-adaptive.cpp)
endif()

if(VISP_HAVE_PIONEER)
  # Add specific build flag to turn off warnings coming from libaria 3rd party
  visp_set_source_file_compile_flag(tutorial-pioneer-robot.cpp -Wno-unused-parameter)
endif()

foreach(cpp ${tutorial_cpp})
  visp_add_target(${cpp})
  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "tutorials")
  endif()
endforeach()
