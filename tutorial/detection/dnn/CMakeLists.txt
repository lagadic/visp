project(tutorial-dnn-object-detection-live)

cmake_minimum_required(VERSION 2.8.12.2)

find_package(VISP REQUIRED visp_detection visp_gui)

# set the list of source files
set(tutorial_cpp
  tutorial-dnn-object-detection-live.cpp)

list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/opencv_face_detector.pbtxt")
list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/opencv_face_detector_uint8.pb")

foreach(cpp ${tutorial_cpp})
  visp_add_target(${cpp})
  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "tutorials")
  endif()
endforeach()

# Copy the data files to the same location than the target
foreach(data ${tutorial_data})
  visp_copy_data(tutorial-dnn-object-detection-live.cpp ${data})
endforeach()

