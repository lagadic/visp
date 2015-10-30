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

# ----------------------------------------------------------------------------
#   Uninstall target, for "make uninstall"
# ----------------------------------------------------------------------------
configure_file(
  cmake/templates/cmake_uninstall.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
  IMMEDIATE @ONLY)

add_custom_target(uninstall
  "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake")

if(ENABLE_SOLUTION_FOLDERS)
  set_target_properties(uninstall PROPERTIES FOLDER "CMakeTargets")
endif()

# ----------------------------------------------------------------------------
#   Doxygen documentation target, for "make visp_doc" and "make html-doc" (to keep compat with previous versions)
# ----------------------------------------------------------------------------
if(DOXYGEN_FOUND)
  add_custom_target(html-doc ${DOXYGEN_EXECUTABLE} ${VISP_DOC_DIR}/config-doxygen) # for compat with previous versions
  add_custom_target(visp_doc ${DOXYGEN_EXECUTABLE} ${VISP_DOC_DIR}/config-doxygen)
  if(ENABLE_SOLUTION_FOLDERS)
    set_target_properties(visp_doc PROPERTIES FOLDER "extra")
    set_target_properties(html-doc PROPERTIES FOLDER "extra")
  endif()
endif()

# ----------------------------------------------------------------------------
#   Tests target, for make visp_tests
# ----------------------------------------------------------------------------
if(BUILD_TESTS)
  add_custom_target(visp_tests)
  if(ENABLE_SOLUTION_FOLDERS)
    set_target_properties(visp_tests PROPERTIES FOLDER "extra")
  endif()
endif()

# ----------------------------------------------------------------------------
#   Tests target, for make visp_examples
# ----------------------------------------------------------------------------
if(BUILD_EXAMPLES)
  add_custom_target(visp_examples)
  if(ENABLE_SOLUTION_FOLDERS)
    set_target_properties(visp_examples PROPERTIES FOLDER "extra")
  endif()
endif()

# ----------------------------------------------------------------------------
#   Tests target, for make visp_demos
# ----------------------------------------------------------------------------
if(BUILD_DEMOS)
  add_custom_target(visp_demos)
  if(ENABLE_SOLUTION_FOLDERS)
    set_target_properties(visp_demos PROPERTIES FOLDER "extra")
  endif()
endif()

# ----------------------------------------------------------------------------
#   Tests target, for make visp_tutorials
# ----------------------------------------------------------------------------
if(BUILD_TUTORIALS)
  add_custom_target(visp_tutorials)
  if(ENABLE_SOLUTION_FOLDERS)
    set_target_properties(visp_tutorials PROPERTIES FOLDER "extra")
  endif()
endif()

# ----------------------------------------------------------------------------
#   Target building all ViSP modules
# ----------------------------------------------------------------------------
add_custom_target(visp_modules)
if(ENABLE_SOLUTION_FOLDERS)
  set_target_properties(visp_modules PROPERTIES FOLDER "extra")
endif()

