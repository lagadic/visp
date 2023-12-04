

# configured documentation tools and intermediate build results
set(BINARY_BUILD_DIR "${CMAKE_CURRENT_BINARY_DIR}/_build")

# Sphinx cache with pickled ReST documents
set(SPHINX_CACHE_DIR "${CMAKE_CURRENT_BINARY_DIR}/_doctrees")

# HTML output directory
set(SPHINX_HTML_DIR "${VISP_DOC_DIR}/python")

#############################################################################
#
# ViSP, open source Visual Servoing Platform software.
# Copyright (C) 2005 - 2023 by Inria. All rights reserved.
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
# ViSP Python bindings module
#
#############################################################################

# Sphinx Template directory
set(SPHINX_TEMPLATE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/_templates")

configure_file(
	"${CMAKE_CURRENT_SOURCE_DIR}/conf.py.in"
	"${BINARY_BUILD_DIR}/conf.py"
	@ONLY)

foreach(module ${python_bound_modules})
  # start string with 2 spaces since its included in autosummary
  string(REPLACE "visp_" "  visp." python_module_name ${module})
  string(APPEND VISP_PYTHON_MODULES_DOC_INCLUDE ${python_module_name} "\n")
endforeach()

configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/api.rst.in"
  "${CMAKE_CURRENT_SOURCE_DIR}/api.rst"
  @ONLY
)

add_custom_target(visp_python_bindings_doc
  COMMAND ${PYTHON3_EXECUTABLE} -m pip install -r "${CMAKE_CURRENT_SOURCE_DIR}/requirements.txt"
	COMMAND sphinx-build
		-b html
		-c "${BINARY_BUILD_DIR}"
		-d "${SPHINX_CACHE_DIR}"
    -j 8
		"${CMAKE_CURRENT_SOURCE_DIR}"
		"${SPHINX_HTML_DIR}"
	COMMENT "Building Sphinx HTML documentation for ViSP's Python bindings"
)

add_dependencies(visp_python_bindings_doc visp_python_bindings_install)
