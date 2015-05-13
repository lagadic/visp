#############################################################################
#
# $Id$
#
# This file is part of the ViSP software.
# Copyright (C) 2005 - 2014 by INRIA. All rights reserved.
# 
# This software is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# ("GPL") version 2 as published by the Free Software Foundation.
# See the file LICENSE.txt at the root directory of this source
# distribution for additional information about the GNU GPL.
#
# For using ViSP with software that can not be combined with the GNU
# GPL, please contact INRIA about acquiring a ViSP Professional 
# Edition License.
#
# See http://www.irisa.fr/lagadic/visp/visp.html for more information.
# 
# This software was developed at:
# INRIA Rennes - Bretagne Atlantique
# Campus Universitaire de Beaulieu
# 35042 Rennes Cedex
# France
# http://www.irisa.fr/lagadic
#
# If you have questions regarding the use of this file, please contact
# INRIA at visp@inria.fr
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

project(demo-wireframe-simulator)

cmake_minimum_required(VERSION 2.8)

find_package(VISP REQUIRED core robot vs)

# SOURCE variable corresponds to the list of all the sources to build binaries.
# The generate binary comes by removing the .cpp extension to
# the source name.
#
# If you want to add/remove a source, modify here
set(demo_cpp
  servoSimu4Points.cpp
  servoSimuCylinder.cpp
  servoSimuSphere.cpp
)

# rule for binary build
foreach(cpp ${demo_cpp})
  # Compute the name of the binary to create
  get_filename_component(bin ${cpp} NAME_WE)

  # From source compile the binary and add link rules
  include_directories(${VISP_INCLUDE_DIRS})
  add_executable(${bin} ${cpp})
  target_link_libraries(${bin} ${VISP_LIBRARIES})
  add_dependencies(visp_demos ${bin})
  if(ENABLE_SOLUTION_FOLDERS)
    set_target_properties(${bin} PROPERTIES FOLDER "demos")
  endif()

  if(UNIX)
    install(TARGETS ${bin}
      DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/visp-${VISP_VERSION}/demo/wireframe-simulator
      PERMISSIONS OWNER_READ GROUP_READ WORLD_READ
      OWNER_EXECUTE GROUP_EXECUTE WORLD_EXECUTE
      OWNER_WRITE)
  else()
    install(TARGETS ${bin}
      DESTINATION demo/wireframe-simulator
      PERMISSIONS OWNER_READ GROUP_READ WORLD_READ
      OWNER_EXECUTE GROUP_EXECUTE WORLD_EXECUTE
      OWNER_WRITE)
  endif()
endforeach()

# Add test
# To run some of these tests don't forget to set VISP_INPUT_IMAGE_PATH
# environment variable to the ViSP test sequences location.
# To get these sequence download ViSP-images.tar.gz from
# http://www.irisa.fr/lagadic/visp/visp.html
add_test(servoSimu4Points servoSimu4Points -d)
add_test(servoSimuCylinder servoSimuCylinder -d)
add_test(servoSimuSphere servoSimuSphere -d)
