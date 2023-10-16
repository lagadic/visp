# cmake_minimum_required(VERSION 3.5)
# project(_visp)

find_package(pybind11 REQUIRED)
find_package(VISP PATHS ${VISP_BINARY_DIR} NO_DEFAULT_PATH)

include_directories(include)
include_directories(${VISP_INCLUDE_DIRS})

pybind11_add_module(_visp THIN_LTO ${python_bindings_cpp_src})

target_link_libraries(_visp PRIVATE ${VISP_LIBRARIES})
add_dependencies(_visp visp_python_bindings_generator_run)
