#############################################################################
#
# ViSP, open source Visual Servoing Platform software.
# Copyright (C) 2005 - 2026 by Inria. All rights reserved.
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
# Try to find RT library material
#
# RT_FOUND
# RT_LIBRARIES
#
#############################################################################


if(NOT UNIX)
  # message("FindRT.cmake: only available for Unix.")
  set(RT_FOUND FALSE)
else()
  set(RT_SEARCH_HINTS
    ${RT_HOME}
    $ENV{RT_HOME}
  )

  find_library(RT_LIBRARY
    NAMES rt
    HINTS ${RT_SEARCH_HINTS}
    PATH_SUFFIXES lib
  )
  #message("DBG RT_LIBRARY=${RT_LIBRARY}")

  if(RT_LIBRARY)
    set(RT_LIBRARIES ${RT_LIBRARY})
    set(RT_FOUND TRUE)
  else()
    set(RT_FOUND FALSE)
  endif()

  mark_as_advanced(RT_LIBRARY)
endif()
