project(tutorial-visual-servo-visa)

cmake_minimum_required(VERSION 2.6)

find_package(VISP REQUIRED visp_core visp_robot visp_vs visp_blob visp_vision visp_io visp_gui)

# To allow vpVisaSocketAdapter class to work with and without ViSP
add_definitions(-DHAVE_VISP)

# set the list of source files
set(tutorial_cpp
  tutorial-visa-viper650-ibvs.cpp
)

foreach(cpp ${tutorial_cpp})
  visp_add_target(${cpp})
  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "tutorials")
  endif()
endforeach()
