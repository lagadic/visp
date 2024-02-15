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

# Declare the cpp source files as explicitely generated so that pybind11_add_module does not look for them when they are not yet created
set_source_files_properties(${python_bindings_cpp_src} PROPERTIES GENERATED TRUE)

pybind11_add_module(_visp ${python_bindings_cpp_src})

# Place library in build/modules/python/bindings dir so that it doesn't pollute lib dir
# This .so file is not treated the same as the others and we shouldn't link against it when compiling in C++
# when installing the python module, pip will look into this subfolder for .so files to copy into the site-packages
file(MAKE_DIRECTORY "${bindings_gen_location}/src")

set_target_properties(_visp PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
)
# With MSVC, the compiled pyd file is placed in a Release/Debug folder
set(build_configs "NONE" "RELEASE" "DEBUG" "RELEASEWITHDEBINFO" "RELWITHDEBINFO")
foreach(imp_config ${build_configs})
set_target_properties(_visp PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY_${imp_config} "${CMAKE_CURRENT_BINARY_DIR}"
)
endforeach()

foreach(visp_lib ${VISP_LIBRARIES})
  get_target_property(dir ${visp_lib} LIBRARY_OUTPUT_DIRECTORY)
  get_target_property(n ${visp_lib} OUTPUT_NAME)

  message("${dir}/${n}")
endforeach()

set_target_properties(_visp PROPERTIES EXCLUDE_FROM_ALL TRUE)

target_include_directories(_visp PUBLIC include) # Include directory containing custom bindings
target_include_directories(_visp PUBLIC ${VISP_INCLUDE_DIRS})
target_link_libraries(_visp PUBLIC ${VISP_LIBRARIES})
add_dependencies(_visp visp_python_bindings_generator_run)

# Setup pip install
if(PYTHON3INTERP_FOUND)
  configure_file("${CMAKE_CURRENT_SOURCE_DIR}/setup.py.in" "${CMAKE_CURRENT_BINARY_DIR}/setup.py" @ONLY)
  add_custom_target( visp_python_bindings_install
    COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_CURRENT_SOURCE_DIR}/visp" "${CMAKE_CURRENT_BINARY_DIR}/visp"
    COMMAND ${PYTHON3_EXECUTABLE} -m pip install  ${_pip_args} "${CMAKE_CURRENT_BINARY_DIR}"
    DEPENDS _visp
  )
endif()
