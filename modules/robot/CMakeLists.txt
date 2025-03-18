# ############################################################################
#
# ViSP, open source Visual Servoing Platform software.
# Copyright (C) 2005 - 2023 by Inria. All rights reserved.
#
# This software is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
# See the file LICENSE.txt at the root directory of this source
# distribution for additional information about the GNU GPL.
#
# For using ViSP with software that can not be combined with the GNU
# GPL, please contact Inria about acquiring a ViSP Professional
# Edition License.
#
# See https://visp.inria.fr for more information.
#
# This software was developed at:
# Inria Rennes - Bretagne Atlantique
# Campus Universitaire de Beaulieu
# 35042 Rennes Cedex
# France
#
# If you have questions regarding the use of this file, please contact
# Inria at visp@inria.fr
#
# This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
# WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
#
# Description:
# ViSP configuration file.
#
# ############################################################################

# Add optional 3rd parties
set(opt_incs "")
set(opt_system_incs "")
set(opt_libs "")
set(opt_libs_private "")

# Warning: when composing the link line, CMake sometimes end up placing multiple
# copies of a given library. This can produce the following link error
# /usr/local/lib/libvirtuose.a: In function `virtCallDLL':
# (.text+0xc30): multiple definition of `virtCallDLL'
# when VIRTUOSE_LIBRARIES is not the first library used by the module.

# Haption Virtuose SDK.
# This 3rd party should be the first used by the module (see warning below)
if(USE_VIRTUOSE)
  list(APPEND opt_system_incs ${VIRTUOSE_INCLUDE_DIRS})
  list(APPEND opt_libs ${VIRTUOSE_LIBRARIES})

  if(USE_THREADS AND RT_FOUND AND DL_FOUND)
    if(CMAKE_THREAD_LIBS_INIT)
      list(APPEND opt_libs "${CMAKE_THREAD_LIBS_INIT}")
    endif()
    list(APPEND opt_libs ${RT_LIBRARIES})
    list(APPEND opt_libs ${DL_LIBRARIES})
  endif()

  if(MSVC)
    # Work around to remove warning LNK4099; PDB vc120.pdb not found with virtuoseDLL.lib
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} /ignore:4099")
  endif()
endif()

if(USE_AFMA4 OR USE_AFMA6 OR USE_VIPER650 OR USE_VIPER850)
  list(APPEND opt_system_incs ${RAW1394_INCLUDE_DIRS})
  list(APPEND opt_system_incs ${CALINUX_INCLUDE_DIRS})
  list(APPEND opt_system_incs ${IRISA_INCLUDE_DIRS})
  list(APPEND opt_libs ${IRISA_LIBRARIES})
  list(APPEND opt_libs ${CALINUX_LIBRARIES})
  list(APPEND opt_libs ${RAW1394_LIBRARIES})
  list(APPEND opt_libs ${RT_LIBRARIES})
endif()

if(USE_UR_RTDE)
  vp_get_external_target(ur_rtde::rtde UR_RTDE_INCLUDE_DIRS UR_RTDE_LIBRARIES)
  list(APPEND opt_system_incs ${UR_RTDE_INCLUDE_DIRS})
  list(APPEND opt_libs ${UR_RTDE_LIBRARIES})
endif()

if(USE_MAVSDK)
  vp_get_external_target(MAVSDK::mavsdk MAVSDK_INCLUDE_DIRS MAVSDK_LIBRARIES)
  list(APPEND opt_system_incs ${MAVSDK_INCLUDE_DIRS})
  list(APPEND opt_libs ${MAVSDK_LIBRARIES})
endif()

if(USE_BICLOPS)
  # No need to export, there is an implementation that allows to use Biclops as private
  include_directories(${BICLOPS_INCLUDE_DIRS})
  list(APPEND opt_libs_private ${BICLOPS_LIBRARIES})
endif()

if(USE_PTU46)
  list(APPEND opt_system_incs ${PTU46_INCLUDE_DIRS})
  list(APPEND opt_libs ${PTU46_LIBRARIES})
endif()

if(USE_JACOSDK)
  list(APPEND opt_system_incs ${JACOSDK_INCLUDE_DIRS})
  list(APPEND opt_libs ${JACOSDK_LIBRARIES})
endif()

if(USE_FLIRPTUSDK)
  list(APPEND opt_system_incs ${FLIRPTUSDK_INCLUDE_DIRS})
  list(APPEND opt_libs ${FLIRPTUSDK_LIBRARIES})
endif()

if(USE_ARIA AND UNIX AND USE_PTHREAD AND RT_FOUND AND DL_FOUND)
  # Under Unix we need Aria, pthread, dl and rt libraries
  list(APPEND opt_system_incs ${ARIA_INCLUDE_DIRS})
  list(APPEND opt_libs ${ARIA_LIBRARIES})
  if(CMAKE_THREAD_LIBS_INIT)
    list(APPEND opt_libs "${CMAKE_THREAD_LIBS_INIT}")
  endif()
  list(APPEND opt_libs ${RT_LIBRARIES})
  list(APPEND opt_libs ${DL_LIBRARIES})
elseif(USE_ARIA AND NOT UNIX)
  list(APPEND opt_system_incs ${ARIA_INCLUDE_DIRS})
  list(APPEND opt_libs ${ARIA_LIBRARIES})
endif()

if(USE_FRANKA)
  vp_get_external_target(${Franka_LIBRARIES} FRANKA_INCLUDE_DIRS FRANKA_LIBRARIES)
  list(APPEND opt_system_incs ${FRANKA_INCLUDE_DIRS})
  list(APPEND opt_libs ${FRANKA_LIBRARIES})
endif()

if(USE_ARSDK)
  list(APPEND opt_system_incs ${ARSDK_INCLUDE_DIRS})
  list(APPEND opt_libs ${ARSDK_LIBRARIES})

  if(USE_FFMPEG)
    list(APPEND opt_system_incs ${FFMPEG_INCLUDE_DIRS})
    list(APPEND opt_libs ${FFMPEG_LIBRARIES})
  endif()
endif()

# for the wireframe simulator we can use optional opengl and coin
if(USE_COIN3D)
  if(WIN32)
    add_definitions("-DCOIN_DLL")
  endif()

  list(APPEND opt_system_incs ${COIN3D_INCLUDE_DIRS})

  # On OSX cmake 2.8 found OpenGL but OPENGL_INCLUDE_DIR was set to NOT_FOUND
  # We add a test to be sure that the OPENGL vars exist.
  if(OPENGL_INCLUDE_DIR)
    list(APPEND opt_system_incs ${OPENGL_INCLUDE_DIR})
  endif()

  if(OPENGL_LIBRARIES)
    list(APPEND opt_libs ${OPENGL_LIBRARIES})
  endif()

  list(APPEND opt_libs ${COIN3D_LIBRARIES})
endif()

if(WITH_QBDEVICE)
  # qbdevice is private
  include_directories(${QBDEVICE_INCLUDE_DIRS})
  list(APPEND opt_libs_private ${QBDEVICE_LIBRARIES})
endif()

if(WITH_TAKKTILE2)
  # reflex-takktile2 is private
  include_directories(${TAKKTILE2_INCLUDE_DIRS})
  list(APPEND opt_libs_private ${TAKKTILE2_LIBRARIES})
endif()

if(WITH_POLOLU)
  # Rapa Pololu Maestro 3rdparty is private
  include_directories(${POLOLU_INCLUDE_DIRS})
  list(APPEND opt_libs_private ${POLOLU_LIBRARIES})
endif()

vp_add_module(robot visp_core OPTIONAL visp_io visp_gui visp_sensor PRIVATE_OPTIONAL ${opt_libs_private})
vp_glob_module_sources()

# copy robot and wireframe simulator data
vp_glob_module_copy_data("src/robot-simulator/arms/*.bnd" data/robot-simulator)
vp_glob_module_copy_data("src/wireframe-simulator/scene/*.bnd" data/wireframe-simulator)
vp_glob_module_copy_data("src/wireframe-simulator/scene/*.sld" data/wireframe-simulator)

vp_module_include_directories(${opt_incs} SYSTEM ${opt_system_incs})
vp_create_module(${opt_libs})
vp_add_tests(
  DEPENDS_ON
    visp_sensor visp_vision visp_blob visp_gui
  CTEST_EXCLUDE_PATH
    bebop2 qbdevice servo-afma4 servo-afma6 servo-franka servo-pixhawk servo-pololu servo-universal-robots
    servo-viper virtuose
)
