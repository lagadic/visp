#############################################################################
#
# ViSP, open source Visual Servoing Platform software.
# Copyright (C) 2005 - 2024 by Inria. All rights reserved.
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
# Try to find OpenCV framework.
#
# Panda3D_FOUND
# Panda3D_INCLUDE_DIRS
# Panda3D_LIBRARIES
#
#############################################################################

set(PANDA3D_INCLUDE_SEARCH_PATHS
  $ENV{Panda3D_DIR}/include
  $ENV{Panda3D_DIR}/built/include
  ${Panda3D_DIR}/include
  ${Panda3D_DIR}/built/include
  /usr/include/panda3d
  /usr/local/include/panda3d
  /Library/Developer/Panda3D/include
)

set(PANDA3D_LIBRARIES_SEARCH_PATHS
  $ENV{Panda3D_DIR}/lib
  $ENV{Panda3D_DIR}/built/lib
  $ENV{Panda3D_DIR}/bin
  ${Panda3D_DIR}/lib
  ${Panda3D_DIR}/built/lib
  ${Panda3D_DIR}/bin
  /usr/lib/panda3d
  /usr/local/lib/panda3d
  /usr/lib/x86_64-linux-gnu/panda3d
  /Library/Developer/Panda3D/lib
)

set(PANDA3D_LIBS
  panda p3framework pandaexpress
  p3dtoolconfig p3dtool p3direct
  #pandaegg
  #p3ffmpeg p3interrogatedb p3tinydisplay p3vision
  #pandaai pandafx pandaphysics pandaskel
)

# Fetch all libraries
set(Panda3D_LIBRARIES "")
set(ALL_LIBS_FOUND TRUE)
foreach(lib_name ${PANDA3D_LIBS})
  find_library(Panda3D_${lib_name}_LIBRARY NAMES "lib${lib_name}" ${lib_name} PATHS ${PANDA3D_LIBRARIES_SEARCH_PATHS})
  if(NOT Panda3D_${lib_name}_LIBRARY)
    set(ALL_LIBS_FOUND FALSE)
  else()
    list(APPEND Panda3D_LIBRARIES ${Panda3D_${lib_name}_LIBRARY})
  endif()

  mark_as_advanced(Panda3D_${lib_name}_LIBRARY)
endforeach()

find_path(Panda3D_INCLUDE_DIRS panda.h PATHS ${PANDA3D_INCLUDE_SEARCH_PATHS})
message(${Panda3D_INCLUDE_DIRS})
include(FindPackageHandleStandardArgs)
# Handle the QUIETLY and REQUIRED arguments and set the Panda3D_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(Panda3D DEFAULT_MSG ALL_LIBS_FOUND Panda3D_INCLUDE_DIRS)

if(Panda3D_FOUND)
  vp_parse_header2(Panda3D "${Panda3D_INCLUDE_DIRS}/pandaVersion.h" PANDA_VERSION_STR)
endif()

mark_as_advanced(Panda3D_INCLUDE_DIRS)
