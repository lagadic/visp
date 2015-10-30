#############################################################################
#
# This file is part of the ViSP software.
# Copyright (C) 2005 - 2015 by Inria. All rights reserved.
#
# This software is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# ("GPL") version 2 as published by the Free Software Foundation.
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
# ViSP overall configuration file. Add extra compilation flags.
#
# Authors:
# Fabien Spindler
#
#############################################################################

macro(vp_add_extra_compilation_flags)
  # By default set release configuration
  if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE String "Choose the type of build, options are: None Debug Release" FORCE)
  endif()

  include(CheckCXXCompilerFlag)
  if(CMAKE_COMPILER_IS_GNUCXX OR MINGW OR CMAKE_CXX_COMPILER_ID MATCHES "Clang") #Not only UNIX but also WIN32 for MinGW

    set(WARNING_ALL "-Wall")
    CHECK_CXX_COMPILER_FLAG(${WARNING_ALL} WARNING_ALL_ALLOWED)
    if(WARNING_ALL_ALLOWED)
#     MESSAGE("Compiler flag ${WARNING_ALL} allowed")
      set(ACTIVATE_WARNING_ALL "ON" CACHE BOOL "activate -Wall flag")
    else()
#     MESSAGE("Compiler flag ${WARNING_ALL} not allowed")
    endif()
  
    set(WARNING_EXTRA "-Wextra")
    CHECK_CXX_COMPILER_FLAG(${WARNING_EXTRA} WARNING_EXTRA_ALLOWED)
    if(WARNING_EXTRA_ALLOWED)
#     MESSAGE("Compiler flag ${WARNING_EXTRA} allowed")
      set(ACTIVATE_WARNING_EXTRA "ON" CACHE BOOL "activate -Wextra flag")
    else()
#     MESSAGE("Compiler flag ${WARNING_EXTRA} not allowed")
    endif()

    set(WARNING_STRICT_OVERFLOW "-Wstrict-overflow=5")
    CHECK_CXX_COMPILER_FLAG(${WARNING_STRICT_OVERFLOW} WARNING_STRICT_OVERFLOW_ALLOWED)
    if(WARNING_STRICT_OVERFLOW_ALLOWED)
#     MESSAGE("Compiler flag ${WARNING_STRICT_OVERFLOW} allowed")
      set(ACTIVATE_WARNING_STRICT_OVERFLOW "OFF" CACHE BOOL "activate -Wstrict-overflow=5 flag")
    else()
#     MESSAGE("Compiler flag ${WARNING_STRICT_OVERFLOW} not allowed")
    endif()

    set(WARNING_FLOAT_EQUAL "-Wfloat-equal")
    CHECK_CXX_COMPILER_FLAG(${WARNING_FLOAT_EQUAL} WARNING_FLOAT_EQUAL_ALLOWED)
    if(WARNING_FLOAT_EQUAL_ALLOWED)
#     MESSAGE("Compiler flag ${WARNING_FLOAT_EQUAL} allowed")
      set(ACTIVATE_WARNING_FLOAT_EQUAL "OFF" CACHE BOOL "activate -Wfloat-equal flag")
    else()
#     MESSAGE("Compiler flag ${WARNING_FLOAT_EQUAL} not allowed")
    endif()

    set(WARNING_SIGN_CONVERSION "-Wsign-conversion")
    CHECK_CXX_COMPILER_FLAG(${WARNING_SIGN_CONVERSION} WARNING_SIGN_CONVERSION_ALLOWED)
    if(WARNING_SIGN_CONVERSION_ALLOWED)
#     MESSAGE("Compiler flag ${WARNING_SIGN_CONVERSION} allowed")
      set(ACTIVATE_WARNING_SIGN_CONVERSION "OFF" CACHE BOOL "activate -Wsign-conversion flag")
    else()
#     MESSAGE("Compiler flag ${WARNING_SIGN_CONVERSION} not allowed")
    endif()

    if(ACTIVATE_WARNING_ALL)
      list(APPEND CMAKE_CXX_FLAGS ${WARNING_ALL})
    else()
      string(REPLACE ${WARNING_ALL} "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
    endif()
    if(ACTIVATE_WARNING_EXTRA)
      list(APPEND CMAKE_CXX_FLAGS ${WARNING_EXTRA})
    else()
      string(REPLACE ${WARNING_EXTRA} "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
    endif()
    if(ACTIVATE_WARNING_STRICT_OVERFLOW)
      list(APPEND CMAKE_CXX_FLAGS ${WARNING_STRICT_OVERFLOW})
      list(APPEND CMAKE_C_FLAGS ${WARNING_STRICT_OVERFLOW})
    else()
      string(REPLACE ${WARNING_STRICT_OVERFLOW} "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
      string(REPLACE ${WARNING_STRICT_OVERFLOW} "" CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")
    endif()
    if(ACTIVATE_WARNING_FLOAT_EQUAL)
      list(APPEND CMAKE_CXX_FLAGS ${WARNING_FLOAT_EQUAL})
    else()
      string(REPLACE ${WARNING_FLOAT_EQUAL} "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
    endif()
    if(ACTIVATE_WARNING_SIGN_CONVERSION)
      list(APPEND CMAKE_CXX_FLAGS ${WARNING_SIGN_CONVERSION})
    else()
      string(REPLACE ${WARNING_SIGN_CONVERSION} "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
    endif()

  elseif(MSVC)
    # Add specific compilation flags for Windows Visual
  
    set(WARNING_ALL "/Wall")
    CHECK_CXX_COMPILER_FLAG(${WARNING_ALL} WARNING_ALL_ALLOWED)
    if(WARNING_ALL_ALLOWED)
      #MESSAGE("Compiler flag ${WARNING_ALL} allowed")
      set(ACTIVATE_WARNING_ALL "OFF" CACHE BOOL "activate /Wall flag")
    else()
      #MESSAGE("Compiler flag ${WARNING_ALL} not allowed")
    endif()
  
    if(ACTIVATE_WARNING_ALL)
      list(APPEND CMAKE_CXX_FLAGS ${WARNING_ALL})
    else()
      string(REPLACE ${WARNING_ALL} "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
    endif()
    if(MSVC80 OR MSVC90 OR MSVC10 OR MSVC11)
      # To avoid compiler warning (level 4) C4571, compile with /EHa if you still want 
  # your catch(...) blocks to catch structured exceptions.
      list(APPEND CMAKE_CXX_FLAGS "/EHa") 
    endif()
  endif()

  # If compiler support symbol visibility, enable it.
  include(CheckCCompilerFlag)
  check_c_compiler_flag(-fvisibility=hidden HAS_VISIBILITY)
  if (HAS_VISIBILITY)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fvisibility=hidden")
  endif()

#  FS no more needed since it is added by default by CMake when build as shared
#  if(UNIX)
#    if(CMAKE_COMPILER_IS_GNUCXX OR CV_ICC)
#      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
#    endif()
#  endif()

  # OpenMP
  if(USE_OPENMP)
    set(VISP_OPENMP_FLAGS "${OpenMP_CXX_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  else()
    set(VISP_OPENMP_FLAGS "")
    if(OpenMP_CXX_FLAGS)
      string(REPLACE ${OpenMP_CXX_FLAGS} "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
    endif()
  endif()

  # C++11
  if(USE_CPP11)
    set(VISP_CPP11_FLAGS "${CPP11_CXX_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CPP11_CXX_FLAGS}")
  else()
    set(VISP_CPP11_FLAGS "")
    if(CPP11_CXX_FLAGS)
      string(REPLACE ${CPP11_CXX_FLAGS} "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
    endif()
  endif()

  if(BUILD_COVERAGE)
    # Add build options for test coverage. Currently coverage is only supported
    # on gcc compiler
    # Because using -fprofile-arcs with shared lib can cause problems like:
    # hidden symbol `__bb_init_func', we add this option only for static
    # library build
    message(STATUS "Add -ftest-coverage -fprofile-arcs compiler options")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -ftest-coverage -fprofile-arcs")
  endif()

  # Remove duplicates compilation flags
  separate_arguments(CMAKE_CXX_FLAGS)
  list(REMOVE_DUPLICATES CMAKE_CXX_FLAGS)
  string(REPLACE ";" " " CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}" CACHE STRING "common C++ build flags" FORCE)
  separate_arguments(CMAKE_C_FLAGS)
  list(REMOVE_DUPLICATES CMAKE_C_FLAGS)
  string(REPLACE ";" " " CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}" CACHE STRING "common C build flags" FORCE)

  #message("CMAKE_CXX_FLAGS : ${CMAKE_CXX_FLAGS}")
endmacro(vp_add_extra_compilation_flags)
