list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/CMake")

# configured documentation tools and intermediate build results
set(BINARY_BUILD_DIR "${CMAKE_CURRENT_BINARY_DIR}/_build")

# Sphinx cache with pickled ReST documents
set(SPHINX_CACHE_DIR "${CMAKE_CURRENT_BINARY_DIR}/_doctrees")

# HTML output directory
set(SPHINX_HTML_DIR "${CMAKE_CURRENT_BINARY_DIR}/html")

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
  @ONLY)

add_custom_target(visp_python_docs
  COMMAND ${Python_EXECUTABLE} -m pip install -r "${CMAKE_CURRENT_SOURCE_DIR}/requirements.txt"
	COMMAND sphinx-build
		-q -b html
		-c "${BINARY_BUILD_DIR}"
		-d "${SPHINX_CACHE_DIR}"
		"${CMAKE_CURRENT_SOURCE_DIR}"
		"${SPHINX_HTML_DIR}"
	COMMENT "Building HTML documentation for ViSP Python doc with Sphinx")

add_dependencies(visp_python_docs visp_python_bindings_install)
