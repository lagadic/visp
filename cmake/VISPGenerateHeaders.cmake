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
# Authors:
# Fabien Spindler
#
#############################################################################

# platform-specific config file
configure_file("${VISP_SOURCE_DIR}/cmake/templates/vpConfig.h.in" "${VISP_INCLUDE_DIR}/visp3/core/vpConfig.h")
install(FILES "${VISP_INCLUDE_DIR}/visp3/core/vpConfig.h"
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/visp3/core
  COMPONENT dev
)

#----------------------------------------------------------------------
# information file
#----------------------------------------------------------------------
configure_file(${VISP_SOURCE_DIR}/cmake/templates/ViSP-third-party.txt.in "${VISP_BINARY_DIR}/ViSP-third-party.txt")

# ----------------------------------------------------------------------------
#  visp_modules.h based on actual modules list
# ----------------------------------------------------------------------------
set(VISP_MODULE_DEFINITIONS_CONFIGMAKE "#ifndef __visp_modules_h__\n#define __visp_modules_h__\n\n")

set(VISP_MOD_LIST ${VISP_MODULES_PUBLIC})
vp_list_sort(VISP_MOD_LIST)
foreach(m ${VISP_MOD_LIST})
  if(m MATCHES "^visp_")
    string(REGEX REPLACE "^visp_" "" m "${m}")
  endif()
  string(TOUPPER "${m}" m)
  set(VISP_MODULE_DEFINITIONS_CONFIGMAKE "${VISP_MODULE_DEFINITIONS_CONFIGMAKE}#define VISP_HAVE_MODULE_${m}\n")
endforeach()

set(VISP_MODULE_DEFINITIONS_CONFIGMAKE "${VISP_MODULE_DEFINITIONS_CONFIGMAKE}\n#endif\n")

configure_file("${VISP_SOURCE_DIR}/cmake/templates/visp_modules.h.in" "${VISP_INCLUDE_DIR}/visp3/visp_modules.h")
install(FILES "${VISP_INCLUDE_DIR}/visp3/visp_modules.h"
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/visp3
  COMPONENT dev
)

# ----------------------------------------------------------------------------
#  install old headers
# ----------------------------------------------------------------------------
file(GLOB old_hdrs "${VISP_INCLUDE_DIR}/visp/*.h")
install(FILES ${old_hdrs}
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/visp
  COMPONENT dev
)
