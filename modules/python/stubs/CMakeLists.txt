
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/setup.py.in" "${CMAKE_CURRENT_BINARY_DIR}/setup.py" @ONLY)
# MANIFEST file is not really configured by CMAKE, but doing it like this ensures that the target will be regenerated if MANIFEST.in is modified
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/MANIFEST.in" "${CMAKE_CURRENT_BINARY_DIR}/MANIFEST.in" COPYONLY)


add_custom_target( visp_python_stubs
  COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_CURRENT_BINARY_DIR}/visp"
  COMMAND ${Python_EXECUTABLE} -m pip install  ${_pip_args} -v "${CMAKE_CURRENT_BINARY_DIR}"
  DEPENDS visp_python_bindings_install
)