get_filename_component(TOOLCHAIN_ROOT_DIR ${CMAKE_TOOLCHAIN_FILE} PATH)

# jpeg
set(ENV{LIBJPEG_DIR} "${TOOLCHAIN_ROOT_DIR}/jpeg")
# png
set(ENV{LIBPNG_DIR} "${TOOLCHAIN_ROOT_DIR}/png")
# pthread
set(ENV{PTHREAD_DIR} "${TOOLCHAIN_ROOT_DIR}/cross/i686-aldebaran-linux-gnu/sysroot/usr")
# usb
set(ENV{LIBUSB_1_DIR} "${TOOLCHAIN_ROOT_DIR}/usb_1")
# v4l2
list(APPEND CMAKE_INCLUDE_PATH "${TOOLCHAIN_ROOT_DIR}/cross/i686-aldebaran-linux-gnu/sysroot/usr/include")
set(ENV{V4L2_DIR} "${TOOLCHAIN_ROOT_DIR}/v4l")
# xml2
set(ENV{XML2_DIR} "${TOOLCHAIN_ROOT_DIR}/xml2")
# zlib
set(ENV{ZLIB_DIR} "${TOOLCHAIN_ROOT_DIR}/zlib")
# zbar
set(ENV{ZBAR_DIR} "${TOOLCHAIN_ROOT_DIR}/zbar")

# OpenCV is detected thanks to cmake/FindMyOpenCV.cmake

mark_as_advanced(CCACHE)


