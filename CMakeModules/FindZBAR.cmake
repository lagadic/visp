#############################################################################
#
# $Id: FindV4L2.cmake 4574 2014-01-09 08:48:51Z fspindle $
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
# Try to find zbar library.
# Once run this will define: 
#
# ZBAR_FOUND
# ZBAR_INCLUDE_DIRS
# ZBAR_LIBRARIES
#
# Authors:
# Fabien Spindler
#
#############################################################################

  
find_path(ZBAR_INCLUDE_DIRS zbar.h
  $ENV{ZBAR_DIR}/include
  /usr/include 
  /usr/local/include 
)

find_library(ZBAR_LIBRARIES
  NAMES zbar
  PATHS 
    $ENV{ZBAR_DIR}/lib
    /usr/lib
    /usr/local/lib
)

if(ZBAR_INCLUDE_DIRS AND ZBAR_LIBRARIES)
  set(ZBAR_FOUND TRUE)
else()
  set(ZBAR_FOUND FALSE)
endif()
  
mark_as_advanced(
  ZBAR_INCLUDE_DIRS
  ZBAR_LIBRARIES
)

