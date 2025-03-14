set(VISP_JAVA_SOURCE_VERSION "" CACHE STRING "Java source version (javac Ant target)")
set(VISP_JAVA_TARGET_VERSION "" CACHE STRING "Java target version (javac Ant target)")
mark_as_advanced(VISP_JAVA_SOURCE_VERSION)
mark_as_advanced(VISP_JAVA_TARGET_VERSION)

file(TO_CMAKE_PATH "$ENV{ANT_DIR}" ANT_DIR_ENV_PATH)
file(TO_CMAKE_PATH "$ENV{ProgramFiles}" ProgramFiles_ENV_PATH)

if(CMAKE_HOST_WIN32)
  set(ANT_NAME ant.bat)
else()
  set(ANT_NAME ant)
endif()

find_host_program(ANT_EXECUTABLE NAMES ${ANT_NAME}
  PATHS "${ANT_DIR_ENV_PATH}/bin" "${ANT_DIR_ENV_PATH}" "${ProgramFiles_ENV_PATH}/apache-ant/bin"
#  NO_DEFAULT_PATH
  )

find_host_program(ANT_EXECUTABLE NAMES ${ANT_NAME})

if(ANT_EXECUTABLE)
  execute_process(COMMAND ${ANT_EXECUTABLE} -version
    RESULT_VARIABLE ANT_ERROR_LEVEL
    OUTPUT_VARIABLE ANT_VERSION_FULL
    OUTPUT_STRIP_TRAILING_WHITESPACE)
  if (ANT_ERROR_LEVEL)
    unset(ANT_EXECUTABLE)
    unset(ANT_EXECUTABLE CACHE)
  else()
    string(REGEX MATCH "[0-9]+.[0-9]+.[0-9]+" ANT_VERSION "${ANT_VERSION_FULL}")
    set(ANT_VERSION "${ANT_VERSION}" CACHE INTERNAL "Detected ant version")

    message(STATUS "Found apache ant: ${ANT_EXECUTABLE} (${ANT_VERSION})")
  endif()
endif()
