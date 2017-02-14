project(tutorial-barcode)

cmake_minimum_required(VERSION 2.6)

find_package(VISP REQUIRED visp_core visp_detection visp_io visp_gui visp_sensor)

set(tutorial_cpp
  tutorial-barcode-detector.cpp
  tutorial-barcode-detector-live.cpp)

set(tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/bar-code.pgm")

foreach(cpp ${tutorial_cpp})
  visp_add_target(${cpp})
  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "tutorials")
  endif()
endforeach()

# Copy the data files to the same location than the target
foreach(data ${tutorial_data})
  visp_copy_data(tutorial-barcode-detector.cpp ${data})
endforeach()

if(VISP_HAVE_ZBAR)
  # Add specific build flag to turn off warnings coming from zbar 3rd party
  visp_set_source_file_compile_flag(tutorial-barcode-detector.cpp -Wno-unused-parameter)
  visp_set_source_file_compile_flag(tutorial-barcode-detector-live.cpp -Wno-unused-parameter)
endif()
