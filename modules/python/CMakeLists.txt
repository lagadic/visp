
if(NOT VISP_DIR)
  return()
endif()


find_package(VISP REQUIRED)

# this avoids non-active conda from getting picked anyway on Windows
set(Python_FIND_REGISTRY LAST)
# Use environment variable PATH to decide preference for Python
set(Python_FIND_VIRTUALENV FIRST)
set(Python_FIND_STRATEGY LOCATION)
find_package(Python 3.7 COMPONENTS Interpreter Development) # TODO: use visp function to find python?
find_package(pybind11 REQUIRED)


message("${VISP_INCLUDE_DIRS}")
# TODO: check for pip
if(Python_FOUND AND pybind11_FOUND) # We have Python and pybind11, we can install module

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
  message("Dirs = ${dirs}")
  string(REPLACE "visp_" "" clean_module_name ${module})
  set(cpp_src "${bindings_gen_location}/src/${clean_module_name}.cpp")
  list(APPEND python_bindings_cpp_src "${cpp_src}")
endforeach()

set(json_config_file "{}")
set(json_config_file_path "${CMAKE_CURRENT_BINARY_DIR}/cmake_config.json")

# Add include directories to config file
set(json_include_dirs "[]")
set(include_dirs_count 0)
foreach(include_dir ${VISP_INCLUDE_DIRS})
  message("Include dir: ${include_dir}")
  string(JSON json_include_dirs SET ${json_include_dirs} "${include_dirs_count}" "\"${include_dir}\"")
  MATH(EXPR include_dirs_count "${include_dirs_count}+1")
endforeach()
string(JSON json_config_file SET ${json_config_file} "include_dirs" "${json_include_dirs}")

# For each bound module, add its headers to config file
foreach(module ${python_bound_modules})
  message("${python_bound_modules}")
  string(REPLACE "visp_" "" clean_module_name ${module})

  string(JSON json_config_file SET ${json_config_file} ${clean_module_name} "{}")

  set(json_header_list "[]")
  set(header_count 0)
  foreach(module_header ${VISP_MODULE_${module}_HEADERS})
    string(JSON json_header_list SET ${json_header_list} "${header_count}" "\"${module_header}\"")
    MATH(EXPR header_count "${header_count}+1")
  endforeach()
  string(JSON json_config_file SET ${json_config_file} ${clean_module_name} "headers" "${json_header_list}")
  #file(TOUCH "${cpp_src}")
endforeach()

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
  COMMAND ${Python_EXECUTABLE} -m visp_python_bindgen.generator --config "${CMAKE_CURRENT_SOURCE_DIR}/config" --build-folder ${bindings_gen_location}
  DEPENDS ${python_bound_modules} ${json_config_file_path} ${config_files} ${python_sources} ${pip_files}
)
add_custom_target(visp_python_bindings_generator_run DEPENDS ${python_bindings_cpp_src})

set(VISP_PYTHON_VERSION "${VISP_VERSION}")
# Step 3: Compile and install bindings as a python package
# First, we compile
add_subdirectory(bindings "${bindings_gen_location}")

# Step 4: Copy stubs dir and install stubs for autocompletion

# Step 5: Build documentation




endif()

#todo : Run generator in an initial pass, depends on ConfigFile and vpConfig.h
