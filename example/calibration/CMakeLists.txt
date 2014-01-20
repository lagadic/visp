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
# ViSP configuration file. 
#
# Authors:
# Fabien Spindler
#
#############################################################################

# SOURCE variable corresponds to the list of all the sources to build binaries.
# The generate binary comes by removing the .cpp extension to
# the source name.
#
# If you want to add/remove a source, modify here
SET (SOURCE
  camera_calibration.cpp
  calibrateTsai.cpp
)

# rule for binary build
FOREACH(source ${SOURCE})
  # Compute the name of the binary to create
  GET_FILENAME_COMPONENT(binary ${source} NAME_WE)

  # From source compile the binary and add link rules
  ADD_EXECUTABLE(${binary} ${source})
  TARGET_LINK_LIBRARIES(${binary} ${VISP_INTERN_LIBRARY} ${VISP_EXTERN_LIBRARIES})

ENDFOREACH(source)

# Add test
# To run these tests automaticaly using the crontab under Unix or the task
# manager under windows, we have to add the -d option to turn off the display
# of acquired images
ADD_TEST(calibrateTsai calibrateTsai)

# copy the data
get_target_property(target_location camera_calibration LOCATION)
get_filename_component(target_location "${target_location}" PATH)
list(APPEND data2copy "${CMAKE_CURRENT_SOURCE_DIR}/default-chessboard.cfg" )
list(APPEND data2copy "${CMAKE_CURRENT_SOURCE_DIR}/default-circles.cfg" )
list(APPEND data2copy "${CMAKE_CURRENT_SOURCE_DIR}/chessboard-01.png" )
list(APPEND data2copy "${CMAKE_CURRENT_SOURCE_DIR}/chessboard-02.png" )
list(APPEND data2copy "${CMAKE_CURRENT_SOURCE_DIR}/chessboard-03.png" )
list(APPEND data2copy "${CMAKE_CURRENT_SOURCE_DIR}/chessboard-04.png" )
list(APPEND data2copy "${CMAKE_CURRENT_SOURCE_DIR}/chessboard-05.png" )
list(APPEND data2copy "${CMAKE_CURRENT_SOURCE_DIR}/circles-01.png" )
list(APPEND data2copy "${CMAKE_CURRENT_SOURCE_DIR}/circles-02.png" )
list(APPEND data2copy "${CMAKE_CURRENT_SOURCE_DIR}/circles-03.png" )
list(APPEND data2copy "${CMAKE_CURRENT_SOURCE_DIR}/circles-04.png" )
list(APPEND data2copy "${CMAKE_CURRENT_SOURCE_DIR}/circles-05.png" )
foreach(data ${data2copy})
  add_custom_command(
    TARGET camera_calibration
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy "${data}" "${target_location}"
  )
endforeach()
