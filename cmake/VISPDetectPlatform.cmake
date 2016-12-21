#############################################################################
#
# This file is part of the ViSP software.
# Copyright (C) 2005 - 2017 by Inria. All rights reserved.
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
# Authors:
# Fabien Spindler
#
#############################################################################

if(CMAKE_CL_64)
    set(MSVC64 1)
endif()

if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  set(CMAKE_COMPILER_IS_GNUCXX 1)
  set(CMAKE_COMPILER_IS_CLANGCXX 1)
endif()
if(CMAKE_C_COMPILER_ID STREQUAL "Clang")
  set(CMAKE_COMPILER_IS_GNUCC 1)
  set(CMAKE_COMPILER_IS_CLANGCC 1)
endif()

if(CMAKE_COMPILER_IS_GNUCXX)
  if(WIN32)
    execute_process(COMMAND ${CMAKE_CXX_COMPILER} -dumpmachine
              OUTPUT_VARIABLE VISP_GCC_TARGET_MACHINE
              OUTPUT_STRIP_TRAILING_WHITESPACE)
    if(VISP_GCC_TARGET_MACHINE MATCHES "amd64|x86_64|AMD64")
      set(MINGW64 1)
    endif()
  endif()
endif()

if(MSVC64 OR MINGW64)
  set(X86_64 1)
elseif(MINGW OR (MSVC AND NOT CMAKE_CROSSCOMPILING))
  set(X86 1)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "amd64.*|x86_64.*|AMD64.*")
  set(X86_64 1)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "i686.*|i386.*|x86.*|amd64.*|AMD64.*")
  set(X86 1)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(arm.*|ARM.*)")
  set(ARM 1)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64.*|AARCH64.*)")
  set(AARCH64 1)
endif()

# Workaround for 32-bit operating systems on 64-bit x86_64 processor
if(X86_64 AND CMAKE_SIZEOF_VOID_P EQUAL 4 AND NOT FORCE_X86_64)
  message(STATUS "sizeof(void) = 4 on x86 / x86_64 processor. Assume 32-bit compilation mode (X86=1)")
  unset(X86_64)
  set(X86 1)
endif()

# ----------------------------------------------------------------------------
# Similar code exist in VISPConfig.cmake
# ----------------------------------------------------------------------------

if(NOT DEFINED VISP_STATIC)
  # look for global setting
  if(NOT DEFINED BUILD_SHARED_LIBS OR BUILD_SHARED_LIBS)
    set(VISP_STATIC OFF)
  else()
    set(VISP_STATIC ON)
  endif()
endif()

if(MSVC)
  if(CMAKE_CL_64)
    set(VISP_ARCH x64)
  else()
    set(VISP_ARCH x86)
  endif()
  if(MSVC_VERSION EQUAL 1400)
    set(VISP_RUNTIME vc8)
  elseif(MSVC_VERSION EQUAL 1500)
    set(VISP_RUNTIME vc9)
  elseif(MSVC_VERSION EQUAL 1600)
    set(VISP_RUNTIME vc10)
  elseif(MSVC_VERSION EQUAL 1700)
    set(VISP_RUNTIME vc11)
  elseif(MSVC_VERSION EQUAL 1800)
    set(VISP_RUNTIME vc12)
  elseif(MSVC_VERSION EQUAL 1900)
    set(VISP_RUNTIME vc14)
  elseif(MSVC_VERSION EQUAL 1910)
    set(VISP_RUNTIME vc15)
  endif()
elseif(MINGW)
  set(VISP_RUNTIME mingw)

  execute_process(COMMAND ${CMAKE_CXX_COMPILER} -dumpmachine
                  OUTPUT_VARIABLE VISP_GCC_TARGET_MACHINE
                  OUTPUT_STRIP_TRAILING_WHITESPACE)
  if(VISP_GCC_TARGET_MACHINE MATCHES "64")
    set(MINGW64 1)
    set(VISP_ARCH x64)
  else()
    set(VISP_ARCH x86)
  endif()
endif()
