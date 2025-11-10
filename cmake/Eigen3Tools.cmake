#############################################################################
#
# ViSP, open source Visual Servoing Platform software.
# Copyright (C) 2005 - 2025 by Inria. All rights reserved.
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
# ViSP configuration file.
#
#############################################################################

# Since Eigen 5.0.0, EIGEN3_INCLUDE_DIRS is not defined anymore.
# We need to get the include dir from the imported target Eigen3::Eigen.
macro(vp_find_eigen3 eigen3_include_dirs eigen3_version)
  if (Eigen3_FOUND)
    # Additional check to be sure that Eigen3 include dir is well detected
    if(NOT ${eigen3_include_dirs})
      get_target_property(imported_incs_ Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)
      if(imported_incs_)
        set(${eigen3_include_dirs} ${imported_incs_})
      endif()
    endif()
    if(NOT ${eigen3_version})
      vp_parse_header("${${eigen3_include_dirs}}/Eigen/src/Core/util/Macros.h" EIGEN3_VERSION_LINES EIGEN_WORLD_VERSION EIGEN_MAJOR_VERSION EIGEN_MINOR_VERSION)
      set(${eigen3_version} "${EIGEN_WORLD_VERSION}.${EIGEN_MAJOR_VERSION}.${EIGEN_MINOR_VERSION}")
    endif()
  endif()
endmacro()
