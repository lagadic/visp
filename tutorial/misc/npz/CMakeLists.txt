cmake_minimum_required(VERSION 3.5)

project(tutorial-npz)

find_package(VISP REQUIRED visp_core visp_gui)

set(tutorial_cpp)

list(APPEND tutorial_cpp tutorial-npz.cpp)
list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/ballons.jpg")

foreach(cpp ${tutorial_cpp})
  visp_add_target(${cpp})
  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "tutorials")
  endif()
endforeach()

# Copy the data files to the same location than the target
foreach(data ${tutorial_data})
  visp_copy_data(tutorial-npz.cpp ${data})
endforeach()
