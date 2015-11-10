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
# ViSP overall configuration file. 
#
# Authors:
# Fabien Spindler
#
#############################################################################

project(example-device-display)

cmake_minimum_required(VERSION 2.6)

find_package(VISP REQUIRED visp_core visp_gui visp_io)

set(example_cpp
  displaySequence.cpp
  displayGDI.cpp
  displayD3D.cpp
  displayGTK.cpp
  displayX.cpp
  displayOpenCV.cpp
  displayXMulti.cpp
)

foreach(cpp ${example_cpp})
  visp_add_target(${cpp})
  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "examples")
  endif()
endforeach()

# Add test
# To run some of these tests don't forget to set VISP_INPUT_IMAGE_PATH
# environment variable to the ViSP test sequences location.
# To get these sequence download ViSP-images.tar.gz from
# http://www.irisa.fr/lagadic/visp/visp.html
add_test(displaySequence displaySequence ${OPTION_TO_DESACTIVE_DISPLAY})
add_test(displayGDI displayGDI -c ${OPTION_TO_DESACTIVE_DISPLAY})
add_test(displayD3D displayD3D -c ${OPTION_TO_DESACTIVE_DISPLAY})
add_test(displayGTK displayGTK -c ${OPTION_TO_DESACTIVE_DISPLAY})
add_test(displayX displayX -c ${OPTION_TO_DESACTIVE_DISPLAY})
add_test(displayOpenCV displayOpenCV -c ${OPTION_TO_DESACTIVE_DISPLAY})
add_test(displayXMulti displayXMulti -c ${OPTION_TO_DESACTIVE_DISPLAY})
