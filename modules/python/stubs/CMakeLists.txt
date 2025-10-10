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
# ViSP Python bindings stubs
#
#############################################################################

configure_file("${CMAKE_CURRENT_SOURCE_DIR}/setup.py.in" "${CMAKE_CURRENT_BINARY_DIR}/setup.py" @ONLY)
# MANIFEST file is not really configured by CMAKE, but doing it like this ensures that the target will be regenerated if MANIFEST.in is modified
#configure_file("${CMAKE_CURRENT_SOURCE_DIR}/MANIFEST.in" "${CMAKE_CURRENT_BINARY_DIR}/MANIFEST.in" COPYONLY)

add_custom_target( visp_python_bindings_stubs
  COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_CURRENT_BINARY_DIR}/visp-stubs"
  COMMAND ${PYTHON3_EXECUTABLE} -m pip install  ${_pip_args} pybind11-stubgen
  COMMAND ${PYTHON3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/run_stub_generator.py --output-root ${CMAKE_CURRENT_BINARY_DIR}
  COMMAND ${PYTHON3_EXECUTABLE} -m pip install  ${_pip_args} "${CMAKE_CURRENT_BINARY_DIR}"
  COMMENT "Generating Python stubs with pybind11-stubgen..."
  DEPENDS visp_python_bindings_install
)
