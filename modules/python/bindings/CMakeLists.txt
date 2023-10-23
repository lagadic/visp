
# Declare the cpp source files as explicitely generated so that pybind11_add_module does not look for them when they are not yet created
set_source_files_properties(${python_bindings_cpp_src} PROPERTIES GENERATED TRUE)

pybind11_add_module(_visp ${python_bindings_cpp_src})

# Place library in binary/visp dir so that it doesn't pollute lib dir
# This .so file is not treated the same as the others and we shouldn't link against it when compiling in C++
# when installing the python module, pip will look into the "visp" subfolder for .so files to copy into the site-packages

file(MAKE_DIRECTORY "${bindings_gen_location}/src")
set_target_properties(_visp PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
)

target_include_directories(_visp PRIVATE include) # Include directory containing custom bindings
target_include_directories(_visp PRIVATE ${VISP_INCLUDE_DIRS})
target_link_libraries(_visp PRIVATE ${VISP_LIBRARIES})
add_dependencies(_visp visp_python_bindings_generator_run)

# Setup pip install
if(Python_FOUND)
  configure_file("${CMAKE_CURRENT_SOURCE_DIR}/setup.py.in" "${CMAKE_CURRENT_BINARY_DIR}/setup.py" @ONLY)
  add_custom_target( visp_python_bindings_install
    COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_CURRENT_SOURCE_DIR}/visp" "${CMAKE_CURRENT_BINARY_DIR}/visp"
    COMMAND ${Python_EXECUTABLE} -m pip install  ${_pip_args} -v "${CMAKE_CURRENT_BINARY_DIR}"
    DEPENDS _visp
  )
endif()