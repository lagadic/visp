#############################################################################
#
# $Id$
#
# Copyright (C) 1998-2010 Inria. All rights reserved.
#
# This software was developed at:
# IRISA/INRIA Rennes
# Projet Lagadic
# Campus Universitaire de Beaulieu
# 35042 Rennes Cedex
# http://www.irisa.fr/lagadic
#
# This file is part of the ViSP toolkit.
#
# This file may be distributed under the terms of the Q Public License
# as defined by Trolltech AS of Norway and appearing in the file
# LICENSE included in the packaging of this file.
#
# Licensees holding valid ViSP Professional Edition licenses may
# use this file in accordance with the ViSP Commercial License
# Agreement provided with the Software.
#
# This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
# WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
#
# Contact visp@irisa.fr if any conditions of this licensing are
# not clear to you.
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
