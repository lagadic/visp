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
# ViSP configuration file. 
#
# Authors:
# Fabien Spindler
#
#############################################################################

project(example-device-framegrabber)

cmake_minimum_required(VERSION 2.6)

find_package(VISP REQUIRED visp_core visp_sensor visp_io visp_gui)

set(example_cpp
  grab1394Two.cpp
  grab1394CMU.cpp
  grabDisk.cpp
  grabV4l2.cpp
  grabDirectShow.cpp
  grabDirectShowMulti.cpp
  grabOpenCV.cpp
  grabOpenCV-2.cpp
)

foreach(cpp ${example_cpp})
  visp_add_target(${cpp})
  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "examples")
  endif()
endforeach()

# Add test
# To run these tests automaticaly using the crontab under Unix or the task
# manager under windows, we have to add the -d option to turn off the display
# of acquired images
#ADD_TEST(grab1394Two      grab1394Two ${OPTION_TO_DESACTIVE_DISPLAY})
#ADD_TEST(grabDisk         grabDisk ${OPTION_TO_DESACTIVE_DISPLAY})
#ADD_TEST(grabV4l2         grabV4l2 ${OPTION_TO_DESACTIVE_DISPLAY})
#ADD_TEST(grabDirectShow   grabDirectShow ${OPTION_TO_DESACTIVE_DISPLAY})
#ADD_TEST(grabDirectShowMulti   grabDirectShowMulti ${OPTION_TO_DESACTIVE_DISPLAY})
#ADD_TEST(grabOpenCV       grabOpenCV ${OPTION_TO_DESACTIVE_DISPLAY})
