# cmake_minimum_required(VERSION 3.5)
# project(_visp)

find_package(VISP PATHS ${VISP_BINARY_DIR} NO_DEFAULT_PATH)
find_package(pybind11 REQUIRED)

set_source_files_properties(${python_bindings_cpp_src} PROPERTIES GENERATED TRUE)

#find_package(VISP REQUIRED)

pybind11_add_module(_visp THIN_LTO ${python_bindings_cpp_src})
target_include_directories(_visp PRIVATE include)
target_include_directories(_visp PRIVATE ${VISP_INCLUDE_DIRS})
message("${VISP_LIBRARIES}")
message("${VISP_INCLUDE_DIRS}")

target_link_libraries(_visp PRIVATE ${VISP_LIBRARIES})
add_dependencies(_visp visp_python_bindings_generator_run)
