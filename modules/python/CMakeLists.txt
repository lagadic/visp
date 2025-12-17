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
# ViSP Python bindings module
#
#############################################################################

# Prevent CMAKE from interpreting this directory as a standard module.
if(NOT VISP_DIR)
  return()
endif()

# As we need all the others modules to already be configured,
# we should configure the python directory by add_subdirectory("modules/python") in the main cmake.
find_package(VISP REQUIRED)

# TODO: check for pip

# Step 1: Generate configuration file
# Define modules for which to generate python bindings
set(python_ignored_modules "visp_python" "visp_java_bindings_generator" "visp_java" )
set(python_bound_modules ${VISP_MODULES_BUILD})
list(REMOVE_ITEM python_bound_modules ${python_ignored_modules})

# Configure the different directories
set(bindgen_package_location "${CMAKE_CURRENT_SOURCE_DIR}/generator")
set(bindings_package_location "${CMAKE_CURRENT_SOURCE_DIR}/bindings")
set(bindings_gen_location "${CMAKE_CURRENT_BINARY_DIR}/bindings")
file(MAKE_DIRECTORY "${bindings_gen_location}/src")
#file(TOUCH "${bindings_gen_location}/src/main.cpp")
set(python_bindings_cpp_src "${bindings_gen_location}/src/main.cpp")

foreach(module ${python_bound_modules})
  get_target_property(dirs "${module}" INCLUDE_DIRECTORIES)
  string(REPLACE "visp_" "" clean_module_name ${module})
  set(cpp_src "${bindings_gen_location}/src/${clean_module_name}.cpp")
  list(APPEND python_bindings_cpp_src "${cpp_src}")
endforeach()

include("${CMAKE_CURRENT_SOURCE_DIR}/GenerateConfig.cmake")

# Step 2: Generate bindings
# First, we install the bindings generator as an editable pip package
# Then, we call it with the configuration files as argument. The .cpp files are generated in the cmake build directory

# Get dependencies of the bindings generator
# We should only run the generator when the config files, the sources or the C++ modules have changed
file(GLOB config_files "${CMAKE_CURRENT_SOURCE_DIR}/config/*.json")
file(GLOB_RECURSE python_sources "${CMAKE_CURRENT_SOURCE_DIR}/generator/visp_python_bindgen/*.py")
set(pip_files "${CMAKE_CURRENT_SOURCE_DIR}/generator/pyproject.toml")

set(bindings_dependencies
  ${python_bound_modules}
  ${json_config_file_path} ${config_files}
  ${python_sources} ${pip_files}
)

# If we have doxygen, we should first generate the XML documentation
# so that the binding stubs and doc is as complete as possible
if(DOXYGEN_FOUND)
  list(APPEND bindings_dependencies visp_doc_xml)
endif()

add_custom_command(
  OUTPUT ${python_bindings_cpp_src}
  COMMAND ${PYTHON3_EXECUTABLE} -m pip install  ${_pip_args} ${bindgen_package_location}
  COMMAND ${PYTHON3_EXECUTABLE} -m visp_python_bindgen.generator --config "${CMAKE_CURRENT_SOURCE_DIR}/config" --build-folder ${bindings_gen_location} --main-config "${json_config_file_path}"
  COMMAND ${PYTHON3_EXECUTABLE} -m pip uninstall -y visp_python_bindgen
  DEPENDS ${bindings_dependencies}
  COMMENT "Installing the python bindings generator and running it..."
)
add_custom_target(
  visp_python_bindings_generator_run
  DEPENDS ${python_bindings_cpp_src}
)

set(VISP_PYTHON_VERSION "${VISP_VERSION}")
set(VISP_PYTHON_BINDINGS_BUILD_PATH "${CMAKE_CURRENT_BINARY_DIR}/bindings")

# Step 3: Compile and install bindings as a python package
add_subdirectory(bindings)

# Step 4: Copy stubs dir and install stubs for autocompletion
add_subdirectory(stubs)

# Global target: compile and install the Python bindings
add_custom_target(
  visp_python_bindings
  DEPENDS visp_python_bindings_stubs
)

# Step 5: Build documentation
if(BUILD_PYTHON_BINDINGS_DOC)
  add_subdirectory(doc)
endif()

# Step 6: Test bindings
add_subdirectory(test)


# Export Variables to parent cmake
set(VISP_PYTHON_BOUND_MODULES "")
foreach(module ${python_bound_modules})
  string(REPLACE "visp_" "" clean_module_name ${module})
  list(APPEND VISP_PYTHON_BOUND_MODULES "${clean_module_name}")
endforeach()
set(VISP_PYTHON_BOUND_MODULES "${VISP_PYTHON_BOUND_MODULES}" PARENT_SCOPE)
set(VISP_PYTHON_GENERATED_CONFIG_FILE "${json_config_file_path}" PARENT_SCOPE)

set(VISP_PYTHON_PACKAGE_VERSION "${VISP_PYTHON_VERSION}" PARENT_SCOPE)
