cmake_minimum_required(VERSION 3.10)

project(apps-calibration-hand-eye)

find_package(VISP REQUIRED visp_vision visp_io OPTIONAL_COMPONENTS visp_gui visp_robot visp_sensor)

set(apps_cpp
  visp-compute-apriltag-poses.cpp
  visp-compute-chessboard-poses.cpp
  visp-compute-eye-in-hand-calibration.cpp
  visp-compute-eye-to-hand-calibration.cpp
)

if(VISP_HAVE_FRANKA)
  list(APPEND apps_cpp visp-acquire-franka-calib-data.cpp)
endif()

if(VISP_HAVE_UR_RTDE)
  list(APPEND apps_cpp visp-acquire-universal-robots-calib-data.cpp)
endif()

foreach(cpp ${apps_cpp})
  visp_add_app(${cpp})
endforeach()

# Copy the data folder to the same location than the target
visp_copy_dir(visp-compute-eye-in-hand-calibration.cpp "${CMAKE_CURRENT_SOURCE_DIR}" data-eye-in-hand)
visp_copy_dir(visp-compute-eye-to-hand-calibration.cpp "${CMAKE_CURRENT_SOURCE_DIR}" data-eye-to-hand)
