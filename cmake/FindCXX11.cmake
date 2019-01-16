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
# Try to find C++11 flags.
#
# CXX11_FOUND - true if C++11 support is detected
# CXX11_CXX_FLAGS - flags to add to the CXX compiler for C++11 support
#
# Authors:
# Fabien Spindler

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED FALSE)
set(CMAKE_CXX_EXTENSIONS OFF) # use -std=c++11 instead of -std=gnu++11
if(CMAKE_CXX11_COMPILE_FEATURES)
  set(CXX11_STANDARD_FOUND ON)
endif()

# Do not uncomment next line, otherwise FindIsNaN.cmake will not use cxx11 flags and you will get a compilation error.
#if(NOT CXX11_STANDARD_FOUND)
  vp_check_compiler_flag(CXX "-std=c++11" HAVE_STD_CXX11_FLAG "${VISP_SOURCE_DIR}/cmake/checks/cxx11.cpp")
  if(HAVE_STD_CXX11_FLAG)
    set(CXX11_CXX_FLAGS "-std=c++11" CACHE STRING "C++ compiler flags for C++11 support")
    mark_as_advanced(CXX11_CXX_FLAGS)
  endif()
#endif()

if(CXX11_STANDARD_FOUND OR CXX11_CXX_FLAGS)
  set(CXX11_FOUND ON)
endif()
