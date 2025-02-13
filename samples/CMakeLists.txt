# ----------------------------------------------------------------------------
#  CMake file for Android samples. See root CMakeLists.txt
#
# ----------------------------------------------------------------------------

install(DIRECTORY android/app DESTINATION ${ANDROID_INSTALL_SAMPLES_DIR})
install(DIRECTORY android/gradle DESTINATION ${ANDROID_INSTALL_SAMPLES_DIR})
install(FILES android/settings.gradle DESTINATION ${ANDROID_INSTALL_SAMPLES_DIR})
install(FILES android/build.gradle DESTINATION ${ANDROID_INSTALL_SAMPLES_DIR})
configure_file("android/local.properties.in" "${ANDROID_TMP_INSTALL_BASE_DIR}/${ANDROID_INSTALL_SAMPLES_DIR}/local.properties" @ONLY)
install(FILES "${ANDROID_TMP_INSTALL_BASE_DIR}/${ANDROID_INSTALL_SAMPLES_DIR}/local.properties" DESTINATION "${ANDROID_INSTALL_SAMPLES_DIR}" COMPONENT samples)
