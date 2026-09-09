:: Setup clang-cl compiler
:: Explicit path is necessary to avoid getting the clang-cl compiler installed on system
set CC="%CONDA_PREFIX%\\Library\\bin\\clang-cl"
set CXX="%CONDA_PREFIX%\\Library\\bin\\clang-cl"
set VISP_EXTRA_CMAKE_ARGS="%VISP_EXTRA_CMAKE_ARGS% -DCMAKE_FIND_ROOT_PATH=%CONDA_PREFIX% -DCMAKE_FIND_ROOT_PATH_MODE_PACKAGE=ONLY -DCMAKE_FIND_ROOT_PATH_MODE_LIBRARY=ONLY -DCMAKE_FIND_ROOT_PATH_MODE_INCLUDE=ONLY"
