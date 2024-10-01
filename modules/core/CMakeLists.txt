#############################################################################
#
# ViSP, open source Visual Servoing Platform software.
# Copyright (C) 2005 - 2023 by Inria. All rights reserved.
#
# This software is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
# See the file LICENSE.txt at the root directory of this source
# distribution for additional information about the GNU GPL.
#
# For using ViSP with software that can not be combined with the GNU
# GPL, please contact Inria about acquiring a ViSP Professional
# Edition License.
#
# See https://visp.inria.fr for more information.
#
# This software was developed at:
# Inria Rennes - Bretagne Atlantique
# Campus Universitaire de Beaulieu
# 35042 Rennes Cedex
# France
#
# If you have questions regarding the use of this file, please contact
# Inria at visp@inria.fr
#
# This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
# WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
#
# Description:
# ViSP configuration file.
#
#############################################################################

#vp_define_module(core)

# Add optional 3rd parties
set(opt_incs "")
set(opt_system_incs "")
set(opt_libs "")
set(opt_libs_private "")

if(WITH_LAPACK)
  # lapack is private
  set(opt_libs_private ${LAPACK_LIBRARIES})
endif()

# Add library ws2_32.a or ws2_32.lib for vpNetwork class
if(WS2_32_FOUND)
  list(APPEND opt_libs ${WS2_32_LIBRARY})
endif()

# OpenCV
if(USE_OPENCV)
  # On win32 since OpenCV 2.4.7 and on OSX with OpenCV 2.4.10 we cannot use OpenCV_LIBS to set ViSP 3rd party libraries.
  # Using OpenCV_LIBS works to build visp library, examples, demos and test thanks to the components,
  # but not tutorials that are stand alone Cmake project that use ViSP as a 3rd party.
  # To be clear OpenCV_LIBS contains opencv_core and not c:\...\opencv_core248.lib full path as requested
  # to use ViSP. This was not the case with OpenCV 2.4.6.
  # For the build of ViSP it works with OpenCV_LIBS: in that case thanks to opencv_core properties, CMake
  # is able to find the real name and location of the libraries.
  # But when ViSP is used as a 3rd party where it should import OpenCV libraries, it doesn't work with
  # OpenCV_LIBS.
  # The solution here is to get the real name of OpenCV libraries thanks to the properties and link
  # with these names.
  # An other way could be to include OpenCVConfig.cmake, but in that case, visp-config and visp.pc
  # will be not able to give the names of OpenCV libraries when used without CMake.
  #message("OpenCV_LIB_COMPONENTS: ${OpenCV_LIB_COMPONENTS}")
  #message("OpenCV_LIBS: ${OpenCV_LIBS}")
  #if(WIN32 AND OpenCV_LIB_COMPONENTS AND OpenCV_VERSION AND OpenCV_VERSION VERSION_GREATER 2.4.6.1)
  if(OpenCV_LIB_COMPONENTS AND OpenCV_VERSION AND OpenCV_VERSION VERSION_GREATER 2.4.6.1)
    # ReleaseWithDebugInfo was requested to detect libopencv-devel under Fedora 20
    # RelWithDebugInfo was requested to detect ros-hydro-opencv2 2.4.9 under Ubuntu 12.04 LTS with ROS hydro
    set(config_ "NONE" "RELEASE" "DEBUG" "RELEASEWITHDEBINFO" "RELWITHDEBINFO")
    set(OpenCV_REQUIRED_LIB_COMPONENTS ${OpenCV_LIB_COMPONENTS})
    vp_list_filterout(OpenCV_REQUIRED_LIB_COMPONENTS "opencv_*")
    # We import only required OpenCV libraries
    list(APPEND OpenCV_REQUIRED_LIB_COMPONENTS "opencv_core" "opencv_imgproc" "opencv_highgui" "opencv_calib3d" "opencv_features2d")
    if(OpenCV_VERSION AND OpenCV_VERSION VERSION_LESS 2.4.8)
      list(APPEND OpenCV_REQUIRED_LIB_COMPONENTS "opencv_legacy")
    endif()

    foreach(component_ ${OpenCV_REQUIRED_LIB_COMPONENTS})
      string(TOUPPER "${component_}" component_UP)
      if(${component_UP}_FOUND)
        foreach(imp_config_ ${config_})
          if(OpenCV_SHARED)
            get_target_property(component_property_${imp_config_}_ ${component_} IMPORTED_IMPLIB_${imp_config_})
            # particular case of opencv_ts that doesn't have an implib
            if(NOT EXISTS "${component_property_${imp_config_}_}")
              get_target_property(component_property_${imp_config_}_ ${component_} IMPORTED_LOCATION_${imp_config_})
            endif()
          else()
            get_target_property(component_property_${imp_config_}_ ${component_} IMPORTED_LOCATION_${imp_config_})
          endif()
          get_target_property(component_property_3rdparty_${imp_config_}_ ${component_} IMPORTED_LINK_INTERFACE_LIBRARIES_${imp_config_})
          #message("component_property_${imp_config_}_: ${component_property_${imp_config_}_}")
          #message("component_property_3rdparty_${imp_config_}_: ${component_property_3rdparty_${imp_config_}_}")
          # Under Unix, there is no specific suffix for OpenCV libraries. If one is found we add it
          # Under Windows, we add the "optimized", "debug" specific keywords
          if(WIN32 AND EXISTS "${component_property_${imp_config_}_}" AND "${imp_config_}" MATCHES "RELEASE") # also valid for RELEASEWITHDEBINFO
            list(APPEND opt_libs optimized "${component_property_${imp_config_}_}")
          elseif(WIN32 AND EXISTS "${component_property_${imp_config_}_}" AND "${imp_config_}" MATCHES "DEBUG")
            list(APPEND opt_libs debug     "${component_property_${imp_config_}_}")
          elseif(EXISTS "${component_property_${imp_config_}_}")
            list(APPEND opt_libs "${component_property_${imp_config_}_}")
          endif()

          if(EXISTS "${component_property_3rdparty_${imp_config_}_}")
            foreach(3rdparty_ ${component_property_3rdparty_${imp_config_}_})
              #message("3rdparty_ ${3rdparty_}")
              list(FIND OpenCV_REQUIRED_LIB_COMPONENTS ${3rdparty_} 3rdparty_is_opencv_component_)
              if(3rdparty_is_opencv_component_ LESS 0)
                #message("${3rdparty_} is not an opencv component")
                get_target_property(3rdparty_opt_location_ ${3rdparty_} IMPORTED_LOCATION_${imp_config_})
                if(NOT EXISTS "${3rdparty_opt_location_}")
                  #message("3rdparty_: ${3rdparty_} location doesn't exist in ${imp_config_}")
                  get_target_property(3rdparty_opt_location_ ${3rdparty_} IMPORTED_LOCATION)
                  #message("3rdparty_: ${3rdparty_} location : ${3rdparty_opt_location_}")
                endif()
                if(EXISTS "${3rdparty_opt_location_}")
                  #message("3rdparty_opt_location_: ${3rdparty_opt_location_} with config ${imp_config_}")
                  if(WIN32 AND "${imp_config_}" MATCHES "RELEASE")
                    #message("is release")
                    list(APPEND opt_libs optimized ${3rdparty_opt_location_})
                  elseif(WIN32 AND "${imp_config_}" MATCHES "DEBUG")
                    list(APPEND opt_libs debug ${3rdparty_opt_location_})
                  else()
                    list(APPEND opt_libs ${3rdparty_opt_location_})
                  endif()
                else()
                  find_library(3rdparty_location_ NAMES ${3rdparty_})
                  mark_as_advanced(3rdparty_location_)
                  if(3rdparty_location_)
                    #message(${3rdparty_location_})
                    list(APPEND opt_libs ${3rdparty_location_}) # should be a system dependency
                  else()
                    list(APPEND opt_libs ${3rdparty_}) # should be a system dependency
                  endif()
                endif()
              endif()
            endforeach()
          endif()
        endforeach()
      endif()
    endforeach()

  elseif(APPLE_FRAMEWORK)
    # Add opencv framework
    list(APPEND opt_libs ${OpenCV_LIBRARIES})    # see FindMyOpenCV.cmake
  elseif(CMAKE_TOOLCHAIN_FILE AND I_AM_A_ROBOT)
    # Add opencv framework from naoqi atom toolchain
    list(APPEND opt_libs ${OpenCV_LIBRARIES})    # see FindMyOpenCV.cmake
  else()
    # this should be an old OpenCV version that doesn't have the previous behavior
    list(APPEND opt_libs ${OpenCV_LIBS})
  endif()
  if(OpenCV_INCLUDE_DIRS)
    # Append OpenCV include dirs as system to avoid warnings coming from OpenCV headers
    list(APPEND opt_incs ${OpenCV_INCLUDE_DIRS})
  endif()
endif(USE_OPENCV)

if(USE_YARP)
  foreach(lib_ ${YARP_LIBRARIES})
    get_target_property(imported_libs_ ${lib_} INTERFACE_LINK_LIBRARIES)
    if(imported_libs_)
      list(APPEND YARP_LIBS ${lib_})
      list(APPEND YARP_IMPORTED_LIBS ${imported_libs_})
    else()
      list(APPEND YARP_LIBS ${lib_})
    endif()

    get_target_property(imported_incs_ ${lib_} INTERFACE_INCLUDE_DIRECTORIES)
    if(imported_incs_)
      list(APPEND YARP_IMPORTED_INCLUDE_DIRS ${imported_incs_})
    endif()
  endforeach()
  vp_list_unique(YARP_IMPORTED_LIBS)
  vp_list_unique(YARP_IMPORTED_INCLUDE_DIRS)

  list(APPEND opt_incs ${YARP_IMPORTED_INCLUDE_DIRS})

  # Work around to add Yarp libraries and also third party libraries requested by Yarp
  list(REVERSE YARP_LIBRARIES) # to start with YARP_init, that depends on YARP_dev, YARP_sig and YARP_OS
  foreach(lib ${YARP_LIBRARIES})
    get_target_property(CONFIGURATIONS ${lib} IMPORTED_CONFIGURATIONS)
    foreach(CONFIGURATION ${CONFIGURATIONS})
      get_target_property(YARP_LIB ${lib} "IMPORTED_LOCATION_${CONFIGURATION}") # Get Yarp full absolute library path and name

      if(WIN32)
        #Work around when YARP is build as shared libraries
        string(REGEX REPLACE ".dll$" ".lib" YARP_LIB ${YARP_LIB})

        if(${CONFIGURATION} STREQUAL "RELEASE")
          list(APPEND VISP_EXTERN_LIBRARIES optimized ${YARP_LIB}) # Append full absolute library path and name
        elseif(${CONFIGURATION} STREQUAL "DEBUG")
          list(APPEND VISP_EXTERN_LIBRARIES debug ${YARP_LIB}) # Append full absolute library path and name
        endif()
      else()
        list(APPEND opt_libs ${YARP_LIB}) # Append full absolute library path and name
      endif()
    endforeach()
  endforeach()
endif(USE_YARP)

# Math: eigen3, gsl, mkl, openblas, atlas, netlib, OpenCV
if(USE_EIGEN3)
  if(EIGEN3_INCLUDE_DIRS)
    list(APPEND opt_system_incs ${EIGEN3_INCLUDE_DIRS})
  else()
    list(APPEND opt_system_incs ${EIGEN3_INCLUDE_DIR})
  endif()
endif()

if(USE_GSL)
  list(APPEND opt_incs ${GSL_INCLUDE_DIRS})
  list(APPEND opt_libs ${GSL_LIBRARIES})
endif()

if(USE_MKL)
  list(APPEND opt_incs ${MKL_INCLUDE_DIRS})
  list(APPEND opt_libs ${MKL_LIBRARIES})
endif()

if(USE_OPENBLAS)
  list(APPEND opt_incs ${OpenBLAS_INCLUDE_DIR})
  list(APPEND opt_libs ${OpenBLAS_LIBRARIES})
endif()

if(USE_ATLAS)
  list(APPEND opt_incs ${Atlas_INCLUDE_DIR})
  list(APPEND opt_libs ${Atlas_LIBRARIES})
endif()

if(USE_NETLIB)
  list(APPEND opt_libs ${NETLIB_LIBRARIES})
endif()

# Misc: xml, pthread, zlib
if(USE_XML2)
  list(APPEND opt_incs ${XML2_INCLUDE_DIRS})
  list(APPEND opt_libs ${XML2_LIBRARIES})
endif()

if(USE_THREADS)
  if(CMAKE_THREAD_LIBS_INIT)
    list(APPEND opt_libs "${CMAKE_THREAD_LIBS_INIT}")
  endif()
endif()

if(USE_ZLIB)
  list(APPEND opt_incs ${ZLIB_INCLUDE_DIRS})
  list(APPEND opt_libs ${ZLIB_LIBRARIES})
endif()

if(USE_OPENMP)
  list(APPEND opt_incs ${OpenMP_CXX_INCLUDE_DIRS})
  # Because there is an explicit link to libpthread location that breaks visp conda package usage on linux
  # we cannot use OpenMP_CXX_LIBRARIES that contains /usr/lib/gcc/x86_64-linux-gnu/11/libgomp.so;/usr/lib/x86_64-linux-gnu/libpthread.a
  # by adding:
  # list(APPEND opt_libs ${OpenMP_CXX_LIBRARIES})
  foreach(lib_ ${OpenMP_CXX_LIB_NAMES})
    if("x${lib_}" STREQUAL "xpthread")
      if(CMAKE_THREAD_LIBS_INIT)
        list(APPEND opt_libs "${CMAKE_THREAD_LIBS_INIT}")
      endif()
    else()
      list(APPEND opt_libs ${OpenMP_${lib_}_LIBRARY})
    endif()
  endforeach()
endif()

if(USE_NLOHMANN_JSON)
  get_target_property(_inc_dirs "nlohmann_json::nlohmann_json" INTERFACE_INCLUDE_DIRECTORIES)
  list(APPEND opt_incs ${_inc_dirs})
endif()

if(USE_PCL)
  list(APPEND opt_incs ${PCL_INCLUDE_DIRS})
  # To ensure to build with VTK and other PCL 3rd parties we are not using PCL_LIBRARIES but PCL_DEPS_INCLUDE_DIRS
  # and PCL_DEPS_LIBRARIES instead
  list(APPEND opt_incs ${PCL_DEPS_INCLUDE_DIRS})
  list(APPEND opt_libs ${PCL_DEPS_LIBRARIES})
endif()

if(WITH_PUGIXML)
  # pugixml is private and provides default XML I/O capabilities
  include_directories(${PUGIXML_INCLUDE_DIRS})
  list(APPEND opt_libs_private ${PUGIXML_LIBRARIES})
endif()

if(WITH_CATCH2)
  # catch2 is private
  include_directories(${CATCH2_INCLUDE_DIRS})
endif()

if(WITH_SIMDLIB)
  # Simd lib is private
  include_directories(${SIMDLIB_INCLUDE_DIRS})
  list(APPEND opt_libs_private ${SIMDLIB_LIBRARIES})
endif()

vp_add_module(core PRIVATE_OPTIONAL ${opt_libs_private} WRAP java)

#-----------------------------------------------------------------------------
# Enable large file support
#-----------------------------------------------------------------------------
if(UNIX OR MINGW)
  if(ANDROID AND (ANDROID_NATIVE_API_LEVEL LESS 21) AND (CMAKE_CXX_COMPILER_ID MATCHES "GNU"))
    # Android NDK build problem: 'mmap' issue with GCC and API<21
  else()
    # This might not catch every possibility
    # For the moment, we define _FILE_OFFSET_BITS=64 only where it is required
    set_source_files_properties(src/tools/file/vpIoTools.cpp PROPERTIES COMPILE_DEFINITIONS "_FILE_OFFSET_BITS=64")
  endif()
endif()

vp_source_group("Src" FILES "${VISP_MODULE_visp_core_BINARY_DIR}/version_string.inc")

vp_glob_module_sources(SOURCES "${VISP_MODULE_visp_core_BINARY_DIR}/version_string.inc")
vp_module_include_directories(${opt_incs} SYSTEM ${opt_system_incs})
vp_create_module(${opt_libs})
vp_create_compat_headers("include/visp3/core/vpConfig.h")
vp_add_tests(CTEST_EXCLUDE_PATH network DEPENDS_ON visp_io visp_gui)

# copy data for testing
vp_glob_module_copy_data("test/math/data/*.pgm" "modules/core" NO_INSTALL)
# copy font
vp_glob_module_copy_data("src/image/private/Rubik-Regular.ttf" "data/font")

if(UNIX)
  vp_set_source_file_compile_flag(src/tools/file/vpIoTools_npy.cpp -Wno-misleading-indentation -Wno-strict-aliasing)
else()
  vp_set_source_file_compile_flag(src/tools/file/vpIoTools_npy.cpp /wd4333)
endif()
