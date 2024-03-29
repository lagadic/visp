############################################################################
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

# configured documentation tools and intermediate build results
set(BINARY_BUILD_DIR "${CMAKE_CURRENT_BINARY_DIR}/_build")

# Sphinx cache with pickled ReST documents
set(SPHINX_CACHE_DIR "${CMAKE_CURRENT_BINARY_DIR}/_doctrees")

# HTML output directory
set(SPHINX_HTML_DIR "${VISP_DOC_DIR}/python")

set(SPHINX_SOURCE_DIR "${${CMAKE_CURRENT_BINARY_DIR}/_src}")

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
  "${BINARY_BUILD_DIR}/api.rst"
  @ONLY
)

set(SPHINX_INPUT_SOURCE_DIRS
  "_static"
  "rst"
  "_templates"
)

set(generated_deps "")

# Copy all the source subdirectories: we're building in the cmake build folder, not in the cmake folder
foreach(source_dir ${SPHINX_INPUT_SOURCE_DIRS})
  set(output_dir "${BINARY_BUILD_DIR}/${source_dir}")
  set(input_dir "${CMAKE_CURRENT_SOURCE_DIR}/${source_dir}")
  file(GLOB_RECURSE files RELATIVE "${input_dir}" "${input_dir}/*")
  foreach(f ${files})
    add_custom_command(
      OUTPUT "${output_dir}/${f}"
      COMMAND ${CMAKE_COMMAND} -E copy_if_different "${input_dir}/${f}" "${output_dir}/${f}"
      MAIN_DEPENDENCY "${input_dir}/${f}"
    )
    list(APPEND generated_deps "${input_dir}/${f}" "${output_dir}/${f}")
  endforeach()
endforeach()

set(output_dir "${BINARY_BUILD_DIR}/examples")
set(input_dir "${CMAKE_CURRENT_SOURCE_DIR}/../examples")
file(GLOB_RECURSE files RELATIVE "${input_dir}" "${input_dir}/*")
foreach(f ${files})
  add_custom_command(
    OUTPUT "${output_dir}/${f}"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different "${input_dir}/${f}" "${output_dir}/${f}"
    MAIN_DEPENDENCY "${input_dir}/${f}"
  )
  list(APPEND generated_deps "${input_dir}/${f}" "${output_dir}/${f}")
endforeach()


set(SPHINX_INPUT_SOURCE_FILES
  "index.rst"
)
foreach(source_file ${SPHINX_INPUT_SOURCE_FILES})
  set(output_file "${BINARY_BUILD_DIR}/${source_file}")
  add_custom_command(
    OUTPUT "${output_file}"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/${source_file}" "${BINARY_BUILD_DIR}"
  )
  list(APPEND generated_deps "${output_file}")
endforeach()

add_custom_target(visp_python_bindings_doc
  COMMAND ${PYTHON3_EXECUTABLE} -m pip install -q -r "${CMAKE_CURRENT_SOURCE_DIR}/requirements.txt"
	COMMAND ${PYTHON3_EXECUTABLE} -m sphinx
		-b html
		-c "${BINARY_BUILD_DIR}"
		-d "${SPHINX_CACHE_DIR}"
    -j auto
    -E
		"${BINARY_BUILD_DIR}"
		"${SPHINX_HTML_DIR}"
  DEPENDS ${generated_deps}
	COMMENT "Building Sphinx HTML documentation for ViSP's Python bindings"
)

add_dependencies(visp_python_bindings_doc visp_python_bindings)
