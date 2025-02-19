:: Activation script

:: Create compile_commands.json for language server
set CMAKE_EXPORT_COMPILE_COMMANDS=1

:: Activate color output with Ninja
set CMAKE_COLOR_DIAGNOSTICS=1

:: Set OS specific extra command CMake options
:: We neeed to force installation of Visp to standard paths instead of <arch>/<vc_version>/ ones done by default
set VISP_EXTRA_CMAKE_ARGS="-DVISP_LIB_INSTALL_PATH:PATH=lib -DVISP_BIN_INSTALL_PATH:PATH=bin -DVISP_CONFIG_INSTALL_PATH:PATH=cmake"
