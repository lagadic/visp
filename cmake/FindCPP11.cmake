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
# Description:
# Try to find C++11 flags.
#
# CPP11_FOUND - true if C++11 support is detected
# CPP11_CXX_FLAGS - flags to add to the CXX compiler for C++11 support
#
# Authors:
# Fabien Spindler

include(CheckCXXCompilerFlag)
include(FindPackageHandleStandardArgs)

set(CPP11_CXX_FLAG_CANDIDATES
  "-std=c++11"
  "-std=gnu++11"
)

# check CXX c++11 compiler flag
foreach(FLAG ${CPP11_CXX_FLAG_CANDIDATES})
  set(SAFE_CMAKE_REQUIRED_FLAGS "${CMAKE_REQUIRED_FLAGS}")
  set(CMAKE_REQUIRED_FLAGS "${FLAG}")
  unset(CPP11_FLAG_DETECTED CACHE)
  message(STATUS "Try C++11 flag = [${FLAG}]")
  CHECK_CXX_COMPILER_FLAG(${FLAG} CPP11_FLAG_DETECTED)
  set(CMAKE_REQUIRED_FLAGS "${SAFE_CMAKE_REQUIRED_FLAGS}")
  if(CPP11_FLAG_DETECTED)
    set(CPP11_CXX_FLAGS_INTERNAL "${FLAG}")
    break()
  endif() 
endforeach()

set(CPP11_CXX_FLAGS "${CPP11_CXX_FLAGS_INTERNAL}"
  CACHE STRING "C++ compiler flags for C++11 support")
# handle the standard arguments for find_package
find_package_handle_standard_args(CPP11 DEFAULT_MSG CPP11_CXX_FLAGS )

mark_as_advanced(
  CPP11_CXX_FLAGS
)
