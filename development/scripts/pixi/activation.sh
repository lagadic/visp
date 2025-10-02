#! /bin/bash
# Activation script

# Create compile_commands.json for language server
export CMAKE_EXPORT_COMPILE_COMMANDS=1

# Activate color output with Ninja
export CMAKE_COLOR_DIAGNOSTICS=1

# Set OS specific extra command CMake options, if any
export VISP_EXTRA_CMAKE_ARGS=""

export VISP_BUILD_DIR=$CONDA_PREFIX/visp-build
export VISP_USE_MATHJAX=${VISP_USE_MATHJAX:=OFF}
export VISP_BUILD_TESTS=${VISP_BUILD_TESTS:=OFF}
export VISP_BUILD_EXAMPLES=${VISP_BUILD_EXAMPLES:=OFF}
export VISP_BUILD_TUTORIALS=${VISP_BUILD_TUTORIALS:=OFF}
export VISP_ENABLE_TEST_WITHOUT_DISPLAY=${VISP_ENABLE_TEST_WITHOUT_DISPLAY:=ON}
