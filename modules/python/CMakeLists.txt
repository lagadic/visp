
set(opt_incs "")
set(opt_libs "")

find_package(Python COMPONENTS Interpreter)
if(Python_FOUND) # We have Python, we can install module

  # Define ViSP module
  vp_add_module(python BINDINGS ${VISP_MODULES_BUILD})

  # Set pip args
  if(DEFINED ENV{VIRTUAL_ENV} OR DEFINED ENV{CONDA_PREFIX})
    set(_pip_args)
  else()
    set(_pip_args "--user")
  endif()
  set(bindgen_package_location "${CMAKE_CURRENT_SOURCE_DIR}/generator")
  # Step 1: Install Bindings generator as an editable package
  add_custom_target(visp_python_bindings_generator_install
      COMMAND ${Python_EXECUTABLE} -m pip install  ${_pip_args} -e ${bindgen_package_location}
  )

  # Step 2: copy static bindings sources to bin directory
  set(bindings_gen_location "${CMAKE_CURRENT_BINARY_DIR}/bindings")
  add_custom_target(visp_python_bindings_generator_run
  COMMAND visp_pybindgen --
  DEPENDS visp_python_bindings_generator_install OPTIONAL visp_doc
  )
  add_custom_command(
      TARGET visp_python_bindings_generator_run
      PRE_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_directory "${root_dir}/${rel_dir}" "$<TARGET_FILE_DIR:${target}>/${rel_dir}"
  )


# Step 3: Compile and install bindings as a python package


# Step 4: Copy stubs dir and install stubs for autocompletion

# Step 5: Build documentation





endif()
