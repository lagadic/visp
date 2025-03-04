project(${the_module}_jar)

set(VISP_JAVA_DIR "${CMAKE_CURRENT_BINARY_DIR}/visp" CACHE INTERNAL "")

file(REMOVE_RECURSE "${VISP_JAVA_DIR}")
file(REMOVE "${VISP_DEPHELPER}/${the_module}_jar_source_copy")

set(java_src_dir "${VISP_JAVA_DIR}/java")
file(MAKE_DIRECTORY "${java_src_dir}")

set(JAR_NAME_WE visp-${VISP_JAVA_LIB_NAME_SUFFIX})
set(JAR_NAME ${JAR_NAME_WE}.jar)
set(VISP_JAR_DIR "${VISP_BINARY_DIR}/bin/" CACHE INTERNAL "")
set(VISP_JAR_FILE "${VISP_BINARY_DIR}/bin/${JAR_NAME}" CACHE INTERNAL "")

vp_copyfiles_append_dir(JAVA_SRC_COPY "${VISP_JAVA_BINDINGS_DIR}/gen/java" "${java_src_dir}")

set(depends gen_visp_java_source "${VISP_DEPHELPER}/gen_visp_java_source")
vp_copyfiles_add_target(${the_module}_jar_source_copy JAVA_SRC_COPY "Copy Java(JAR) source files" ${depends})
set(depends ${the_module}_jar_source_copy "${VISP_DEPHELPER}/${the_module}_jar_source_copy")

if(VISP_JAVA_SOURCE_VERSION)
  set(VISP_ANT_JAVAC_EXTRA_ATTRS "${VISP_ANT_JAVAC_EXTRA_ATTRS} source=\"${VISP_JAVA_SOURCE_VERSION}\"")
endif()
if(VISP_JAVA_TARGET_VERSION)
  set(VISP_ANT_JAVAC_EXTRA_ATTRS "${VISP_ANT_JAVAC_EXTRA_ATTRS} target=\"${VISP_JAVA_TARGET_VERSION}\"")
endif()

if(VISP_JAVA_SDK_BUILD_TYPE STREQUAL "ANT")
  file(MAKE_DIRECTORY "${VISP_JAVA_DIR}/build/classes")

  configure_file("${CMAKE_CURRENT_SOURCE_DIR}/build.xml.in" "${VISP_JAVA_DIR}/build.xml" @ONLY)
  list(APPEND depends "${VISP_JAVA_DIR}/build.xml")

  vp_cmake_byproducts(__byproducts BYPRODUCTS "${VISP_JAR_FILE}")
  add_custom_command(OUTPUT "${VISP_DEPHELPER}/${the_module}_jar"
      ${__byproducts}  # required for add_custom_target() by ninja
      COMMAND ${ANT_EXECUTABLE} -noinput -k jar
      COMMAND ${CMAKE_COMMAND} -E touch "${VISP_DEPHELPER}/${the_module}_jar"
      WORKING_DIRECTORY "${VISP_JAVA_DIR}"
      DEPENDS ${depends}
      COMMENT "Generating ${JAR_NAME}"
  )
  add_custom_target(${the_module}_jar DEPENDS "${VISP_DEPHELPER}/${the_module}_jar")
elseif(VISP_JAVA_SDK_BUILD_TYPE STREQUAL "JAVA")
  configure_file("${CMAKE_CURRENT_SOURCE_DIR}/MANIFEST.MF.in" "${VISP_JAVA_DIR}/MANIFEST.MF" @ONLY)
  list(APPEND depends "${VISP_JAVA_DIR}/MANIFEST.MF")

  vp_cmake_byproducts(__byproducts BYPRODUCTS "${VISP_JAVA_DIR}/java_sources")
  add_custom_command(OUTPUT "${VISP_DEPHELPER}/${the_module}_jar"
      BYPRODUCTS ${__byproducts}  # required for add_custom_target() by ninja
      DEPENDS ${depends}
      COMMAND ${CMAKE_COMMAND} -E touch "${VISP_DEPHELPER}/${the_module}_jar"
      COMMAND ${CMAKE_COMMAND}
      -D VISP_JAVA_DIR="${VISP_JAVA_DIR}/java"
      -D OUTPUT="${VISP_JAVA_DIR}/java_sources"
      -P "${CMAKE_CURRENT_SOURCE_DIR}/list_java_sources.cmake"
  )

  add_custom_target(${the_module}_jar_sources
    DEPENDS "${VISP_DEPHELPER}/${the_module}_jar"
  )

  list(APPEND CMAKE_JAVA_COMPILE_FLAGS -encoding utf-8 ${VISP_EXTRA_JAVA_COMPILE_FLAGS})

  add_jar(${the_module}_jar
          SOURCES "@${VISP_JAVA_DIR}/java_sources"
          MANIFEST "${VISP_JAVA_DIR}/MANIFEST.MF"
          OUTPUT_NAME "${JAR_NAME_WE}"
          OUTPUT_DIR "${VISP_JAR_DIR}")

  add_dependencies(${the_module}_jar ${the_module}_jar_sources)
else()
  vp_assert(0)
endif()


install(FILES ${VISP_JAR_FILE} OPTIONAL DESTINATION ${VISP_JAR_INSTALL_PATH} COMPONENT java)

add_dependencies(${the_module} ${the_module}_jar)

if(BUILD_DOCS)
  if(OPENCV_JAVA_SDK_BUILD_TYPE STREQUAL "ANT")
    add_custom_command(OUTPUT "${VISP_DEPHELPER}/${the_module}doc"
      COMMAND ${ANT_EXECUTABLE} -noinput -k javadoc
      COMMAND ${CMAKE_COMMAND} -E touch "${VISP_DEPHELPER}/${the_module}doc"
      WORKING_DIRECTORY "${VISP_JAVA_DIR}"
      DEPENDS ${depends}
      COMMENT "Generating Javadoc"
    )
    add_custom_target(${the_module}doc DEPENDS "${VISP_DEPHELPER}/${the_module}doc")

    install(DIRECTORY ${VISP_BINARY_DIR}/doc/doxygen/html/javadoc
      DESTINATION "${VISP_DOC_INSTALL_PATH}/html"
      COMPONENT "docs" OPTIONAL
      ${compatible_MESSAGE_NEVER}
    )
  elseif(OPENCV_JAVA_SDK_BUILD_TYPE STREQUAL "JAVA")
    set(Java_JAVADOC_EXECUTABLE ${Java_JAVADOC_EXECUTABLE} -encoding utf-8)

    # create_javadoc produces target ${_target}_javadoc
    create_javadoc(${the_module}
      FILES "@${VISP_JAVA_DIR}/java_sources"
      SOURCEPATH "${VISP_JAVA_DIR}/java"
      INSTALLPATH "${VISP_JAVADOC_DESTINATION}"
      WINDOWTITLE "ViSP ${VISP_VERSION} Java documentation"
      DOCTITLE "ViSP Java documentation (${VISP_VERSION})"
      VERSION TRUE
    )
    add_dependencies(${the_module}_javadoc ${the_module}_jar_sources)
    add_custom_target(${the_module}doc DEPENDS ${the_module}_javadoc)

    install(DIRECTORY ${VISP_BINARY_DIR}/doc/doxygen/html/javadoc/${the_module}/
      DESTINATION "${VISP_DOC_INSTALL_PATH}/html/javadoc"
      COMPONENT "docs" OPTIONAL
      ${compatible_MESSAGE_NEVER}
    )
  else()
    vp_assert(0)
  endif()

  set(CMAKE_DOXYGEN_JAVADOC_NODE
    "<tab type=\"user\" url=\"./javadoc/index.html\" title=\"Java documentation\"/>"
    CACHE INTERNAL "Link to the Java documentation") # set to the cache to make it global
  add_custom_target(doxygen_javadoc DEPENDS ${the_module}doc)
  add_dependencies(visp_docs ${the_module}doc)  # TODO: Do check whether visp needs/has such docs. Dont copy blindly
else()
  unset(CMAKE_DOXYGEN_JAVADOC_NODE CACHE)
endif()
