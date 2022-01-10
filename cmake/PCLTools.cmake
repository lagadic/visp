#############################################################################
#
# ViSP, open source Visual Servoing Platform software.
# Copyright (C) 2005 - 2021 by Inria. All rights reserved.
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
# See http://visp.inria.fr for more information.
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
# Authors:
# Fabien Spindler
#
#############################################################################

if(USE_PCL)
  if(PCL_FOUND)
    mark_as_advanced(PCL_DIR)
    mark_as_advanced(PCL_APPS_LIBRARY)
    mark_as_advanced(PCL_APPS_LIBRARY_DEBUG)
    mark_as_advanced(pkgcfg_lib_FLANN_flann)
    mark_as_advanced(pkgcfg_lib_FLANN_flann_cpp)
    mark_as_advanced(pkgcfg_lib_FLANN_lz4)
    mark_as_advanced(pkgcfg_lib_GLEW_GLEW)
    mark_as_advanced(pkgcfg_lib_PC_OPENNI2_DummyDev)
    mark_as_advanced(pkgcfg_lib_PC_OPENNI2_OniFile)
    mark_as_advanced(pkgcfg_lib_PC_OPENNI2_PS1080)
    mark_as_advanced(pkgcfg_lib_PC_USB_10_usb-1.0)
    mark_as_advanced(pkgcfg_lib_PC_OPENNI2_DummyDevice)
    foreach(component ${PCL_TO_FIND_COMPONENTS})
      string(TOUPPER "${component}" COMPONENT)
      if(PCL_${COMPONENT}_INCLUDE_DIR)
        mark_as_advanced(PCL_${COMPONENT}_INCLUDE_DIR)
      endif()
    endforeach()
  endif()

  mark_as_advanced(EIGEN_INCLUDE_DIRS)

  mark_as_advanced(FLANN_INCLUDE_DIR)
  mark_as_advanced(FLANN_INCLUDE_DIRS)
  mark_as_advanced(FLANN_LIBRARY)
  mark_as_advanced(FLANN_LIBRARY_DEBUG)
  mark_as_advanced(FLANN_LIBRARY_DEBUG_SHARED) # Requested on macOS with pcl 1.12.1
  mark_as_advanced(FLANN_LIBRARY_DEBUG_STATIC) # Requested on macOS with pcl 1.12.1
  mark_as_advanced(FLANN_LIBRARY_SHARED)       # Requested on macOS with pcl 1.12.1
  mark_as_advanced(FLANN_LIBRARY_STATIC)       # Requested on macOS with pcl 1.12.1

  mark_as_advanced(HDF5_C_LIBRARY_dl)          # Requested on macOS with pcl 1.12.1
  mark_as_advanced(HDF5_C_LIBRARY_hdf5)        # Requested on macOS with pcl 1.12.1
  mark_as_advanced(HDF5_C_LIBRARY_hdf5_hl)     # Requested on macOS with pcl 1.12.1
  mark_as_advanced(HDF5_C_LIBRARY_m)           # Requested on macOS with pcl 1.12.1
  mark_as_advanced(HDF5_C_LIBRARY_sz)          # Requested on macOS with pcl 1.12.1
  mark_as_advanced(HDF5_C_LIBRARY_z)           # Requested on macOS with pcl 1.12.1

  mark_as_advanced(ICU_INCLUDE_DIR)            # Requested on macOS with pcl 1.12.1

  mark_as_advanced(LZMA_INCLUDE_DIR)           # Requested on macOS with pcl 1.12.1
  mark_as_advanced(LZMA_LIBRARY)               # Requested on macOS with pcl 1.12.1

  mark_as_advanced(QHULL_INCLUDE_DIRS)
  mark_as_advanced(QHULL_LIBRARY)
  mark_as_advanced(QHULL_LIBRARY_DEBUG)
  mark_as_advanced(Qhull_DIR)                  # Requested on macOS with pcl 1.12.1

  mark_as_advanced(Qt5OpenGL_DIR)              # Requested on macOS with pcl 1.12.1
  mark_as_advanced(Qt5QmlModels_DIR)           # Requested on macOS with pcl 1.12.1
  mark_as_advanced(Qt5Qml_DIR)                 # Requested on macOS with pcl 1.12.1
  mark_as_advanced(Qt5Quick_DIR)               # Requested on macOS with pcl 1.12.1
  mark_as_advanced(Qt5_DIR)                    # Requested on macOS with pcl 1.12.1

  mark_as_advanced(OPENNI2_INCLUDE_DIR)
  mark_as_advanced(OPENNI2_INCLUDE_DIRS)
  mark_as_advanced(OPENNI2_LIBRARY)

  mark_as_advanced(OPENNI_INCLUDE_DIR)
  mark_as_advanced(OPENNI_INCLUDE_DIRS)
  mark_as_advanced(OPENNI_LIBRARY)

  mark_as_advanced(USB_10_INCLUDE_DIR)
  mark_as_advanced(USB_10_LIBRARY)

  mark_as_advanced(Boost_THREAD_LIBRARY_RELEASE)        # Requested on Ubuntu 20.04
  mark_as_advanced(Boost_DATE_TIME_LIBRARY_RELEASE)     # Requested on macOS with pcl 1.12.1
  mark_as_advanced(Boost_FILESYSTEM_LIBRARY_RELEASE)    # Requested on macOS with pcl 1.12.1
  mark_as_advanced(Boost_INCLUDE_DIR)                   # Requested on macOS with pcl 1.12.1
  mark_as_advanced(Boost_IOSTREAMS_LIBRARY_RELEASE)     # Requested on macOS with pcl 1.12.1
  mark_as_advanced(Boost_SERIALIZATION_LIBRARY_RELEASE) # Requested on macOS with pcl 1.12.1
  mark_as_advanced(Boost_SYSTEM_LIBRARY_RELEASE)        # Requested on macOS with pcl 1.12.1

  mark_as_advanced(libusb_INCLUDE_DIR)                  # Requested on macOS with pcl 1.12.1
  mark_as_advanced(netCDF_DIR)                          # Requested on macOS with pcl 1.12.1
  mark_as_advanced(pugixml_DIR)                         # Requested on macOS with pcl 1.12.1
endif()
