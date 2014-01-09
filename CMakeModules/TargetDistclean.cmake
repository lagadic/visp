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
# Add custom target distclean: cleans and removes cmake generated files, etc.
#
# Authors:
# Fabien Spindler
#
#############################################################################

ADD_CUSTOM_TARGET( distclean  DEPENDS clean)

SET(DISTCLEAN_FILES
  cmake.check_cache
  */cmake.check_cache
  cmake_install.cmake
  */cmake_install.cmake
  */*/cmake_install.cmake
  CMakeCache.txt
  core core.*
  gmon
  *~ 
  *%
  SunWS_cache
  ii_files
  )

## for 1.8.x:
ADD_CUSTOM_COMMAND(
  TARGET distclean
  COMMAND 
  ${CMAKE_COMMAND} -E remove ${DISTCLEAN_FILES}
  COMMENT
  )
