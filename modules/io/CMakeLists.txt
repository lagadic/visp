#############################################################################
#
# ViSP, open source Visual Servoing Platform software.
# Copyright (C) 2005 - 2019 by Inria. All rights reserved.
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
# See http://visp.inria.fr for more information.
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
# Authors:
# Fabien Spindler
#
#############################################################################

# Add optional 3rd parties
set(opt_incs "")
set(opt_libs "")

# Image i/o: jpeg, png, OpenCV (already included by core)
if(USE_JPEG)
  list(APPEND opt_incs ${JPEG_INCLUDE_DIR})
  list(APPEND opt_libs ${JPEG_LIBRARIES})
  # If Qt3 and libjpeg are used, we add here QT_CLEAN_NAMESPACE define to
  # handle a compilation error "conflicting declaration 'typedef long int INT32'
  if(VISP_HAVE_QT3_FOUND)
    add_definitions("-DQT_CLEAN_NAMESPACE")
  endif()
endif()

if(USE_PNG)
  list(APPEND opt_incs ${PNG_INCLUDE_DIR})
  list(APPEND opt_libs ${PNG_LIBRARIES})
  add_definitions(${PNG_DEFINITIONS})
endif()

# TODO:
#if(WITH_STBIMAGE)
  # stb_image is private
  include_directories(${STBIMAGE_INCLUDE_DIRS})
#endif()

if(WITH_CATCH2)
  # catch2 is private
  include_directories(${CATCH2_INCLUDE_DIRS})
endif()

# simdlib is always enabled since it contains fallback code to plain C++ code
# Simd lib is private
include_directories(${SIMDLIB_INCLUDE_DIRS})

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
    list(APPEND OpenCV_REQUIRED_LIB_COMPONENTS "opencv_videoio" "opencv_imgcodecs")

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

#  elseif(APPLE_FRAMEWORK)
#    # Add opencv framework
#    list(APPEND opt_libs ${OpenCV_LIBRARIES})    # see FindMyOpenCV.cmake
#  elseif(CMAKE_TOOLCHAIN_FILE AND I_AM_A_ROBOT)
#    # Add opencv framework from naoqi atom toolchain
#    list(APPEND opt_libs ${OpenCV_LIBRARIES})    # see FindMyOpenCV.cmake
#  else()
#    # this should be an old OpenCV version that doesn't have the previous behavior
#    list(APPEND opt_libs ${OpenCV_LIBS})
  endif()
  if(OpenCV_INCLUDE_DIRS)
    list(APPEND opt_incs ${OpenCV_INCLUDE_DIRS})
  endif()
endif(USE_OPENCV)

if(ANDROID)
  vp_add_module(io visp_core)
else()
  vp_add_module(io visp_core WRAP java)
endif()

vp_glob_module_sources()
vp_module_include_directories(${opt_incs})
vp_create_module(${opt_libs})
vp_add_tests()

vp_set_source_file_compile_flag(src/tools/vpParseArgv.cpp -Wno-strict-overflow)
vp_set_source_file_compile_flag(src/image/vpImageIo.cpp -Wno-missing-field-initializers) # since stb_image_write.h update from v1.13 to v1.16
