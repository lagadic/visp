project(apps-calibration)

cmake_minimum_required(VERSION 3.5)

find_package(VISP REQUIRED visp_vision visp_io OPTIONAL_COMPONENTS visp_gui visp_robot visp_sensor)

set(apps_cpp
  visp-compute-chessboard-poses.cpp
  visp-compute-hand-eye-calibration.cpp
)

if(VISP_HAVE_FRANKA)
  list(APPEND apps_cpp visp-acquire-franka-calib-data.cpp)
endif()

if(VISP_HAVE_UR_RTDE)
  list(APPEND apps_cpp visp-acquire-universal-robots-calib-data.cpp)
endif()

file(GLOB apps_data "*.yaml" "*.py" "camera.xml" "*.png")

foreach(cpp ${apps_cpp})
  visp_add_app(${cpp})
endforeach()

if(VISP_HAVE_REALSENSE2)
  # Add specific build flag to turn off warnings coming from librealsense 3rd party and its dependencies
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "/wd4244")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "/wd4267")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-deprecated-copy")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-deprecated-declarations")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-ignored-qualifiers")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-overloaded-virtual")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-pessimizing-move")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-reorder")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-sign-compare")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-unused-function")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-unused-parameter")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-unqualified-std-cast-call")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-missing-field-initializers")

  visp_set_source_file_compile_flag(visp-acquire-franka-calib-data.cpp ${CXX_FLAGS_MUTE_WARNINGS})
  visp_set_source_file_compile_flag(visp-acquire-universal-robots-calib-data.cpp ${CXX_FLAGS_MUTE_WARNINGS})
endif()

# Copy the data files to the same location than the target
foreach(data ${apps_data})
  visp_copy_data(visp-compute-hand-eye-calibration.cpp ${data})
endforeach()
