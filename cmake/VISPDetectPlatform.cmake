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
# Authors:
# Fabien Spindler
#
#############################################################################

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
