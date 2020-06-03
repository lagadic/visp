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
# ViSP overall configuration file. Add extra compilation flags.
#
# Authors:
# Fabien Spindler
#
#############################################################################

set(VISP_EXTRA_C_FLAGS "")
set(VISP_EXTRA_CXX_FLAGS "")

macro(add_extra_compiler_option option)
  if(CMAKE_BUILD_TYPE)
    set(CMAKE_TRY_COMPILE_CONFIGURATION ${CMAKE_BUILD_TYPE})
  endif()
  vp_check_flag_support(CXX "${option}" _varname "")
  if(_varname)
    list(APPEND VISP_EXTRA_CXX_FLAGS ${option})
  endif()

  vp_check_flag_support(C "${option}" _varname "")
  if(_varname)
    list(APPEND VISP_EXTRA_C_FLAGS ${option})
  endif()
endmacro()

macro(add_extra_compiler_option_enabling option var_enabling flag)
  if(CMAKE_BUILD_TYPE)
    set(CMAKE_TRY_COMPILE_CONFIGURATION ${CMAKE_BUILD_TYPE})
  endif()
  vp_check_flag_support(CXX "${option}" _cxx_varname "")
  if(_cxx_varname)
    set(__msg "Activate ${option} compiler flag")
    set(${var_enabling} ${flag} CACHE BOOL ${__msg})
  endif()
  if(${${var_enabling}})
    list(APPEND VISP_EXTRA_CXX_FLAGS ${option})
  else()
    vp_list_filterout(VISP_EXTRA_CXX_FLAGS ${option})
  endif()

  vp_check_flag_support(C "${option}" _c_varname "")
  if(${${var_enabling}} AND _c_varname)
    list(APPEND VISP_EXTRA_C_FLAGS ${option})
  else()
    vp_list_filterout(VISP_EXTRA_C_FLAGS ${option})
  endif()
endmacro()

if(CMAKE_COMPILER_IS_GNUCXX OR MINGW OR CMAKE_CXX_COMPILER_ID MATCHES "Clang") #Not only UNIX but also WIN32 for MinGW
  add_extra_compiler_option_enabling(-Wall               ACTIVATE_WARNING_ALL             ON)
  add_extra_compiler_option_enabling(-Wextra             ACTIVATE_WARNING_EXTRA           ON)
  add_extra_compiler_option_enabling(-Wstrict-overflow=5 ACTIVATE_WARNING_STRICT_OVERFLOW OFF)
  add_extra_compiler_option_enabling(-Wfloat-equal       ACTIVATE_WARNING_FLOAT_EQUAL     OFF)
  add_extra_compiler_option_enabling(-Wsign-conversion   ACTIVATE_WARNING_SIGN_CONVERSION OFF)
  add_extra_compiler_option_enabling(-Wshadow            ACTIVATE_WARNING_SHADOW          OFF)
elseif(MSVC)
  # Add specific compilation flags for Windows Visual
  add_extra_compiler_option_enabling(/Wall               ACTIVATE_WARNING_ALL             OFF)
  if(MSVC80 OR MSVC90 OR MSVC10 OR MSVC11 OR MSVC14)
    # To avoid compiler warning (level 4) C4571, compile with /EHa if you still want
    # your catch(...) blocks to catch structured exceptions.
    add_extra_compiler_option("/EHa")
  endif()
endif()

if(USE_OPENMP)
  add_extra_compiler_option("${OpenMP_CXX_FLAGS}")
endif()

if((VISP_CXX_STANDARD EQUAL VISP_CXX_STANDARD_11) AND CXX11_CXX_FLAGS)
  add_extra_compiler_option("${CXX11_CXX_FLAGS}")
elseif((VISP_CXX_STANDARD EQUAL VISP_CXX_STANDARD_14) AND CXX14_CXX_FLAGS)
  add_extra_compiler_option("${CXX14_CXX_FLAGS}")
elseif((VISP_CXX_STANDARD EQUAL VISP_CXX_STANDARD_17) AND CXX17_CXX_FLAGS)
  add_extra_compiler_option("${CXX17_CXX_FLAGS}")
endif()

if(BUILD_COVERAGE)
  add_extra_compiler_option("-ftest-coverage -fprofile-arcs")
endif()

if(CMAKE_COMPILER_IS_GNUCXX)
  add_extra_compiler_option(-fvisibility=hidden)

  if(ENABLE_AVX AND X86_64)
    add_extra_compiler_option(-mavx)
  else()
    if(ENABLE_SSE2)
      add_extra_compiler_option(-msse2)
    elseif(X86 OR X86_64)
      add_extra_compiler_option(-mno-sse2)
    endif()

    if(ENABLE_SSE3)
      add_extra_compiler_option(-msse3)
    elseif(X86 OR X86_64)
      #add_extra_compiler_option(-mno-sse3)
    endif()

    if(ENABLE_SSSE3)
      add_extra_compiler_option(-mssse3)
    elseif(X86 OR X86_64)
      add_extra_compiler_option(-mno-ssse3)
    endif()
  endif()

  if(X86 AND NOT IOS)
    add_extra_compiler_option(-ffloat-store) #to not use the x87 FPU on x86, see PR #442 for more information
  endif()
endif()

if(MSVC AND X86_64)
  if(ENABLE_AVX AND NOT MSVC_VERSION LESS 1600)
    add_extra_compiler_option("/arch:AVX")
  endif()
endif()

if(UNIX)
  if(CMAKE_COMPILER_IS_GNUCXX)
    add_extra_compiler_option(-fPIC) # Is needed for ANDROID too.
  endif()
endif()

if(DEFINED WINRT_8_1)
  add_extra_compiler_option(/ZW) # do not use with 8.0
endif()

if(MSVC)
  # Remove unreferenced functions: function level linking
  list(APPEND VISP_EXTRA_CXX_FLAGS "/Gy")
  if(NOT MSVC_VERSION LESS 1400)
    # Avoid build error C1128
    list(APPEND VISP_EXTRA_CXX_FLAGS "/bigobj")
  endif()
endif()

# Add user supplied extra options (optimization, etc...)
vp_list_unique(VISP_EXTRA_C_FLAGS)

vp_list_unique(VISP_EXTRA_CXX_FLAGS)
vp_list_remove_separator(VISP_EXTRA_C_FLAGS)
vp_list_remove_separator(VISP_EXTRA_CXX_FLAGS)

set(VISP_EXTRA_C_FLAGS      "${VISP_EXTRA_C_FLAGS}"       CACHE INTERNAL "Extra compiler options for C sources")
set(VISP_EXTRA_CXX_FLAGS    "${VISP_EXTRA_CXX_FLAGS}"     CACHE INTERNAL "Extra compiler options for C++ sources")

#combine all "extra" options
set(CMAKE_C_FLAGS           "${CMAKE_C_FLAGS} ${VISP_EXTRA_C_FLAGS}")
set(CMAKE_CXX_FLAGS         "${CMAKE_CXX_FLAGS} ${VISP_EXTRA_CXX_FLAGS}")
