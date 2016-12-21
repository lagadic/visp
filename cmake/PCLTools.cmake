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
# ViSP configuration file.
#
# Authors:
# Fabien Spindler
#
#############################################################################

if(USE_PCL)
  if(PCL_FOUND)
    mark_as_advanced(PCL_DIR)
    mark_as_advanced(PCL_APPS_LIBRARY)
    mark_as_advanced(PCL_APPS_LIBRARY_DEBUG)
    foreach(component ${PCL_TO_FIND_COMPONENTS})
      string(TOUPPER "${component}" COMPONENT)
      if(PCL_${COMPONENT}_INCLUDE_DIR)
        mark_as_advanced(PCL_${COMPONENT}_INCLUDE_DIR)
      endif()
    endforeach()
  endif()

  if(EIGEN_FOUND)
    mark_as_advanced(EIGEN_INCLUDE_DIRS)
  endif()

  if(FLANN_FOUND)
    mark_as_advanced(FLANN_INCLUDE_DIRS)
    mark_as_advanced(FLANN_LIBRARY)
    mark_as_advanced(FLANN_LIBRARY_DEBUG)
  endif()

  if(QHULL_FOUND)
    mark_as_advanced(QHULL_INCLUDE_DIRS)
    mark_as_advanced(QHULL_LIBRARY)
    mark_as_advanced(QHULL_LIBRARY_DEBUG)
  endif()
  
  mark_as_advanced(OPENNI2_INCLUDE_DIRS)
  mark_as_advanced(OPENNI2_LIBRARY)

  mark_as_advanced(OPENNI_INCLUDE_DIRS)
  mark_as_advanced(OPENNI_LIBRARY)
  
endif()