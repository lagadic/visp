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
# Try to find C99 flags.
#
# C99_FOUND - true if C99 support is detected
# C99_C_FLAGS - flags to add to the C compiler for C99 support

include(CheckCCompilerFlag)
include(FindPackageHandleStandardArgs)

set(C99_C_FLAG_CANDIDATES
  "-std=c99"
  "-std=gnu99"
)

# check C C99 compiler flag
foreach(FLAG ${C99_C_FLAG_CANDIDATES})
  set(SAFE_CMAKE_REQUIRED_FLAGS "${CMAKE_REQUIRED_FLAGS}")
  set(CMAKE_REQUIRED_FLAGS "${FLAG}")
  unset(C99_FLAG_DETECTED CACHE)
  message(STATUS "Try C99 flag = [${FLAG}]")
  CHECK_C_COMPILER_FLAG(${FLAG} C99_FLAG_DETECTED)
  set(CMAKE_REQUIRED_FLAGS "${SAFE_CMAKE_REQUIRED_FLAGS}")
  if(C99_FLAG_DETECTED)
    set(C99_C_FLAGS_INTERNAL "${FLAG}")
    break()
  endif() 
endforeach()

set(C99_C_FLAGS "${C99_C_FLAGS_INTERNAL}")

# handle the standard arguments for find_package
find_package_handle_standard_args(C99 DEFAULT_MSG C99_C_FLAGS )

mark_as_advanced(
  C99_C_FLAGS
)
