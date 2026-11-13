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

if(INSTALL_PYTHON_BINDINGS)
  # Packaging mode: generate the stubs from the build tree without pip,
  # then install them next to the bindings with the CMake install target
  set(_stubs_gen_dir "${CMAKE_CURRENT_BINARY_DIR}/gen")
  set(_stubs_dir "${CMAKE_CURRENT_BINARY_DIR}/visp")
  set(_stubs_stamp "${CMAKE_CURRENT_BINARY_DIR}/stubs.stamp")
  file(GLOB_RECURSE _visp_python_package_sources "${bindings_package_location}/visp/*.py")

  add_custom_command(
    OUTPUT "${_stubs_stamp}"
    # The visp package has to be importable from the build tree: copy its Python sources next to _visp
    COMMAND ${CMAKE_COMMAND} -E copy_directory "${bindings_package_location}/visp" "${VISP_PYTHON_BINDINGS_BUILD_PATH}/visp"
    COMMAND ${CMAKE_COMMAND} -E rm -rf "${_stubs_gen_dir}" "${_stubs_dir}"
    COMMAND ${PYTHON3_EXECUTABLE} "${CMAKE_CURRENT_SOURCE_DIR}/generate_stubs.py"
            --output-root "${_stubs_gen_dir}"
            --target "${_stubs_dir}"
            --bindings-path "${VISP_PYTHON_BINDINGS_BUILD_PATH}"
    COMMAND ${CMAKE_COMMAND} -E touch "${_stubs_stamp}"
    DEPENDS _visp "${CMAKE_CURRENT_SOURCE_DIR}/generate_stubs.py" ${_visp_python_package_sources}
    COMMENT "Generating Python stubs..."
    VERBATIM
  )
  add_custom_target(visp_python_bindings_stubs ALL DEPENDS "${_stubs_stamp}")

  install(DIRECTORY "${_stubs_dir}/"
    DESTINATION "${VISP_PYTHON_INSTALL_DIR}/visp"
    COMPONENT python
    FILES_MATCHING PATTERN "*.pyi"
  )
else()
  configure_file("${CMAKE_CURRENT_SOURCE_DIR}/setup.py" "${CMAKE_CURRENT_BINARY_DIR}/setup.py" COPYONLY)
  configure_file("${CMAKE_CURRENT_SOURCE_DIR}/generate_stubs.py" "${CMAKE_CURRENT_BINARY_DIR}/generate_stubs.py" COPYONLY)
  configure_file("${CMAKE_CURRENT_SOURCE_DIR}/pyproject.toml.in" "${CMAKE_CURRENT_BINARY_DIR}/pyproject.toml" @ONLY)

  add_custom_target( visp_python_bindings_stubs
    COMMAND ${PYTHON3_EXECUTABLE} -m pip install  ${_pip_args} "${CMAKE_CURRENT_BINARY_DIR}"
    COMMENT "Generating Python stubs..."
    DEPENDS visp_python_bindings_install
  )
endif()
