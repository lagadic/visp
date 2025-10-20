:: Activation script

:: Create compile_commands.json for language server
set CMAKE_EXPORT_COMPILE_COMMANDS=1

:: Activate color output with Ninja
set CMAKE_COLOR_DIAGNOSTICS=1

:: Set OS specific extra command CMake options
:: We neeed to force installation of Visp to standard paths instead of <arch>/<vc_version>/ ones done by default
set VISP_EXTRA_CMAKE_ARGS="-DVISP_LIB_INSTALL_PATH:PATH=lib -DVISP_BIN_INSTALL_PATH:PATH=bin -DVISP_CONFIG_INSTALL_PATH:PATH=cmake"
set VISP_BUILD_DIR=%CONDA_PREFIX%\visp-build

if not defined VISP_BUILD_TESTS (set VISP_BUILD_TESTS=OFF)
if not defined VISP_BUILD_EXAMPLES (set VISP_BUILD_EXAMPLES=OFF)
if not defined VISP_USE_MATHJAX (set VISP_USE_MATHJAX=OFF)
if not defined VISP_BUILD_TUTORIALS (set VISP_BUILD_TUTORIALS=OFF)
if not defined VISP_ENABLE_TEST_WITHOUT_DISPLAY (set VISP_ENABLE_TEST_WITHOUT_DISPLAY=ON)
if not defined VISP_BUILD_COVERAGE (set VISP_BUILD_COVERAGE=OFF)
if not defined VISP_BUILD_DEPRECATED_FUNCTIONS (set VISP_BUILD_DEPRECATED_FUNCTIONS=ON)
