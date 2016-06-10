set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_VERSION 1)
set(CMAKE_SYSTEM_PROCESSOR arm)

# Cross toolchain
set(ARM_RASPBERRY_CROSSCHAIN $ENV{HOME}/soft/raspberrypi/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian/bin CACHE PATH "Raspberry cross toolchain location")

find_program(CMAKE_C_COMPILER NAMES ${RPI_CROSS_TOOL}/arm-linux-gnueabihf-gcc)
find_program(CMAKE_CXX_COMPILER NAMES ${RPI_CROSS_TOOL}/arm-linux-gnueabihf-g++)

# Cross-compilation cannot succeed if ARM_RASPBERRY_ROOTFS is not set
set(ARM_RASPBERRY_ROOTFS $ENV{HOME}/soft/raspberrypi/rootfs CACHE PATH "Raspberry cross compilation system root (/usr, /lib) location")

if(EXISTS ${ARM_RASPBERRY_ROOTFS}/usr/include/arm-linux-gnueabihf)
  include_directories(${ARM_RASPBERRY_ROOTFS}/usr/include/arm-linux-gnueabihf)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-psabi")
set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} -Wno-psabi")

set(CMAKE_FIND_ROOT_PATH ${ARM_RASPBERRY_ROOTFS})

set(CMAKE_SKIP_RPATH TRUE CACHE BOOL "If set, runtime paths are not added when using shared libraries." )

set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# Add stuff to crosscompile tests/demos/tutorials/examples
set(__linkdir ${ARM_RASPBERRY_ROOTFS}/lib/arm-linux-gnueabihf ${ARM_RASPBERRY_ROOTFS}/usr/lib ${ARM_RASPBERRY_ROOTFS}/usr/lib/arm-linux-gnueabihf )
foreach(dir ${__linkdir})
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath-link,${dir}")
endforeach()
