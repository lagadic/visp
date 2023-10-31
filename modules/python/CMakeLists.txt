

# Prevent CMAKE from interpreting this directory as a standard module.
if(NOT VISP_DIR)
  return()
endif()

# As we need all the others modules to already be configured,
# we should configure the python directory by add_subdirectory("modules/python") in the main cmake.
find_package(VISP REQUIRED)


# TODO: check for pip

# Set pip args
if(DEFINED ENV{VIRTUAL_ENV} OR DEFINED ENV{CONDA_PREFIX})
  set(_pip_args)
else()
  set(_pip_args "--user")
endif()

# Step 1: Generate configuration file
# Define modules for which to generate python bindings
set(python_ignored_modules "visp_python" "visp_java_bindings_generator" "visp_java")
set(python_bound_modules ${VISP_MODULES_BUILD})
list(REMOVE_ITEM python_bound_modules ${python_ignored_modules})

# Define ViSP module


# Configure
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

set(json_config_file "{}")
set(json_config_file_path "${CMAKE_CURRENT_BINARY_DIR}/cmake_config.json")

# Paths to important directories
string(JSON json_config_file SET ${json_config_file} "xml_doc_path" "\"${VISP_DOC_DIR}/xml\"")
string(JSON json_config_file SET ${json_config_file} "build_dir" "\"${CMAKE_BINARY_DIR}\"")
string(JSON json_config_file SET ${json_config_file} "source_dir" "\"${CMAKE_SOURCE_DIR}\"")

# Add include directories to config file
set(json_include_dirs "[]")
set(include_dirs_count 0)
foreach(include_dir ${VISP_INCLUDE_DIRS})
  string(JSON json_include_dirs SET ${json_include_dirs} "${include_dirs_count}" "\"${include_dir}\"")
  MATH(EXPR include_dirs_count "${include_dirs_count}+1")
endforeach()
string(JSON json_config_file SET ${json_config_file} "include_dirs" "${json_include_dirs}")



# For each bound module, add its headers and dependencies to config file
set(json_modules "{}")
foreach(module ${python_bound_modules})
  string(REPLACE "visp_" "" clean_module_name ${module})

  string(JSON json_modules SET ${json_modules} ${clean_module_name} "{}")

  set(json_header_list "[]")
  set(header_count 0)
  foreach(module_header ${VISP_MODULE_${module}_HEADERS})
    string(JSON json_header_list SET ${json_header_list} "${header_count}" "\"${module_header}\"")
    MATH(EXPR header_count "${header_count}+1")
  endforeach()
  string(JSON json_modules SET ${json_modules} ${clean_module_name} "headers" "${json_header_list}")
  set(json_deps_list "[]")
  set(dep_count 0)
  foreach(dep ${VISP_MODULE_${module}_DEPS})
    string(REPLACE "visp_" "" clean_dep ${dep})
    string(JSON json_deps_list SET ${json_deps_list} "${dep_count}" "\"${clean_dep}\"")
    MATH(EXPR dep_count "${dep_count}+1")
  endforeach()
  string(JSON json_modules SET ${json_modules} ${clean_module_name} "dependencies" "${json_deps_list}")
endforeach()
string(JSON json_config_file SET ${json_config_file} "modules" ${json_modules})

# See https://github.com/cpredef/predef/tree/master for compiler/OS specific #defines
set(json_defines "{}")
string(JSON json_defines SET ${json_defines} "__cplusplus" "${VISP_CXX_STANDARD}")
# Compiler
if(CMAKE_COMPILER_IS_GNUCXX)
  string(REPLACE "." ";" GCC_VERSION_LIST ${CMAKE_CXX_COMPILER_VERSION})
  list(GET GCC_VERSION_LIST 0 GCC_MAJOR)
  list(GET GCC_VERSION_LIST 1 GCC_MINOR)
  list(GET GCC_VERSION_LIST 2 GCC_PATCH)

  string(JSON json_defines SET ${json_defines} "__GNUC__" "${GCC_MAJOR}")
  string(JSON json_defines SET ${json_defines} "__GNUC_MINOR__" "${GCC_MINOR}")
  string(JSON json_defines SET ${json_defines} "__GNUC_PATCHLEVEL__" "${GCC_PATCH}")
endif()
if(CMAKE_COMPILER_IS_CLANGCXX)
  string(REPLACE "." ";" CLANG_VERSION_LIST ${CMAKE_CXX_COMPILER_VERSION})
  list(GET CLANG_VERSION_LIST 0 CLANG_MAJOR)
  list(GET CLANG_VERSION_LIST 1 CLANG_MINOR)
  list(GET CLANG_VERSION_LIST 2 CLANG_PATCH)

  string(JSON json_defines SET ${json_defines} "__clang__" "${CLANG_MAJOR}")
  string(JSON json_defines SET ${json_defines} "__clang_minor__" "${CLANG_MINOR}")
  string(JSON json_defines SET ${json_defines} "__clang_patchlevel__" "${CLANG_PATCH}")
  string(JSON json_defines SET ${json_defines} "__clang_version__" "${CMAKE_CXX_COMPILER_VERSION}")
endif()

if(MSVC)
  string(JSON json_defines SET ${json_defines} "_MSC_VER" "${MSVC_VERSION}")
endif()

if(MINGW)
  string(JSON json_defines SET ${json_defines} "__MINGW32__" "null")
endif()
# OS
if(WIN32)
  string(JSON json_defines SET ${json_defines} "_WIN32" "null")
endif()
if(UNIX)
  string(JSON json_defines SET ${json_defines} "__linux__" "null")
  string(JSON json_defines SET ${json_defines} "__unix__" "null")
  string(JSON json_defines SET ${json_defines} "_unix" "null")
endif()
if(APPLE)
  string(JSON json_defines SET ${json_defines} "__APPLE__" "null")
  string(JSON json_defines SET ${json_defines} "__MACH__" "null")
endif()

string(JSON json_config_file SET ${json_config_file} "defines" ${json_defines})




file(WRITE ${json_config_file_path} "${json_config_file}")


# Step 2: Generate bindings
# First, we install the bindings generator as an editable pip package
# Then, we call it with the configuration files as argument. The .cpp files are generated in the cmake build directory

# Get dependencies of the bindings generator
# We should only run the generator when the config files, the sources or the C++ modules have changed
file(GLOB config_files "${CMAKE_CURRENT_SOURCE_DIR}/config/*.json")
file(GLOB_RECURSE python_sources "${CMAKE_CURRENT_SOURCE_DIR}/generator/visp_python_bindgen/*.py")
set(pip_files "${CMAKE_CURRENT_SOURCE_DIR}/generator/pyproject.toml")

add_custom_command(
  OUTPUT ${python_bindings_cpp_src}
  COMMAND ${Python_EXECUTABLE} -m pip install  ${_pip_args} -v ${bindgen_package_location}
  COMMAND ${Python_EXECUTABLE} -m visp_python_bindgen.generator --config "${CMAKE_CURRENT_SOURCE_DIR}/config" --build-folder ${bindings_gen_location} --main-config "${json_config_file_path}"
  DEPENDS ${python_bound_modules} ${json_config_file_path} ${config_files} ${python_sources} ${pip_files}
)
add_custom_target(
  visp_python_bindings_generator_run
  DEPENDS ${python_bindings_cpp_src}

)

set(VISP_PYTHON_VERSION "${VISP_VERSION}")
# Step 3: Compile and install bindings as a python package
add_subdirectory(bindings)


# Step 4: Copy stubs dir and install stubs for autocompletion
add_subdirectory(stubs)

# Step 5: Build documentation
if(BUILD_PYTHON_DOC)
  add_subdirectory(docs)
endif()




# Export Variables to parent cmake
set(VISP_PYTHON_BOUND_MODULES "")
foreach(module ${python_bound_modules})
  string(REPLACE "visp_" "" clean_module_name ${module})
  list(APPEND VISP_PYTHON_BOUND_MODULES "${clean_module_name}")
endforeach()
set(VISP_PYTHON_BOUND_MODULES "${VISP_PYTHON_BOUND_MODULES}" PARENT_SCOPE)
set(VISP_PYTHON_GENERATED_CONFIG_FILE "${json_config_file_path}" PARENT_SCOPE)

set(VISP_PYTHON_PACKAGE_VERSION "${VISP_PYTHON_VERSION}" PARENT_SCOPE)
