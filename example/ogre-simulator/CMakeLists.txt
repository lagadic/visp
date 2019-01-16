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

project(example-ogre-simulator)

cmake_minimum_required(VERSION 2.6)

find_package(VISP REQUIRED visp_core visp_vision visp_ar visp_blob visp_io visp_gui)

set(example_cpp
  AROgre.cpp
  AROgreBasic.cpp
)

foreach(cpp ${example_cpp})
  visp_add_target(${cpp})

  if(VISP_HAVE_OGRE)
    # Add specific build flag to turn off warnings coming from libogre and libois 3rd party
    visp_set_source_file_compile_flag(${cpp} -Wno-unused-parameter -Wno-unused-but-set-parameter -Wno-overloaded-virtual)
  endif()

  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "examples")
  endif()
endforeach()
