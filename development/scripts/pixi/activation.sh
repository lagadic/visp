#! /bin/bash
# Activation script

# Create compile_commands.json for language server
export CMAKE_EXPORT_COMPILE_COMMANDS=1

# Activate color output with Ninja
export CMAKE_COLOR_DIAGNOSTICS=1

# Set OS specific extra command CMake options, if any
export VISP_EXTRA_CMAKE_ARGS=""

export VISP_BUILD_TESTS=${VISP_BUILD_TESTS:=OFF}
export VISP_BUILD_DIR=$CONDA_PREFIX/visp-build
