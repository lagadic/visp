project(tutorial-robot-flir-ptu)

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

find_package(VISP REQUIRED visp_core visp_robot visp_sensor visp_vision visp_gui visp_vs visp_visual_features visp_detection)

set(tutorial_cpp
  tutorial-flir-ptu-ibvs.cpp
)

set(example_data
  ${CMAKE_CURRENT_SOURCE_DIR}/eMc.yaml)

if(VISP_HAVE_FLYCAPTURE)
  # Add specific build flag to turn off warnings coming from Flir Flycapture 3rd party
  visp_set_source_file_compile_flag(tutorial-flir-ptu-ibvs.cpp -Wno-unknown-pragmas -Wno-ignored-qualifiers)
endif()

foreach(cpp ${tutorial_cpp})
  visp_add_target(${cpp})
  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "tutorials")
  endif()
endforeach()

# Copy the data files to the same location than the target
foreach(data ${example_data})
  visp_copy_data(tutorial-flir-ptu-ibvs.cpp ${data})
endforeach()
