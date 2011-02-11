#############################################################################
#
# $Id$
#
# This file is part of the ViSP software.
# Copyright (C) 2005 - 2011 by INRIA. All rights reserved.
# 
# This software is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# ("GPL") version 2 as published by the Free Software Foundation.
# See the file LICENSE.txt at the root directory of this source
# distribution for additional information about the GNU GPL.
#
# For using ViSP with software that can not be combined with the GNU
# GPL, please contact INRIA about acquiring a ViSP Professional 
# Edition License.
#
# See http://www.irisa.fr/lagadic/visp/visp.html for more information.
# 
# This software was developed at:
# INRIA Rennes - Bretagne Atlantique
# Campus Universitaire de Beaulieu
# 35042 Rennes Cedex
# France
# http://www.irisa.fr/lagadic
#
# If you have questions regarding the use of this file, please contact
# INRIA at visp@inria.fr
# 
# This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
# WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
#
# Description:
# ViSP packaging configuration file.
#
# Authors:
# Fabien Spindler
#
#############################################################################

# To install the generated debian package use the gdebi-gtk package 
# installer on Ubuntu:
# gdebi-gtk libvisp-2.6.0-dev.deb
#
# Since there is no opencv package compatible with ViSP for the moment
# we desactivate the opencv usage and we activate the soqt usage since 
# there exist a soqt package
# cmake -DUSE_OPENCV=OFF -DUSE_SOQT=ON

set(BUILD_SHARED_LIBS CACHE FORCE "Build ViSP with shared libraries." ON)
list(APPEND CPACK_GENERATOR RPM)

set(CPACK_PACKAGE_FILE_NAME "libvisp-${VISP_VERSION}-dev")
set(CPACK_RPM_PACKAGE_GROUP "Development/Libraries")
#set(CPACK_RPM_PACKAGE_ARCHITECTURE "i386")

set(CPACK_RPM_PACKAGE_REQUIRES "cmake >= 2.6, libX11-devel >= 1.3, gsl-devel >= 1.13, libxml2-devel >= 2.7.6, libpng-devel >= 2:1.2.44, libjpeg-devel >= 6b-46, Coin2-devel >= 2.5.0, SoQt-devel >= 1.5.0")

SET(CPACK_RPM_PACKAGE_DESCRIPTION "${CPACK_PACKAGE_DESCRIPTION_SUMMARY}\r\rViSP stands for Visual Servoing Platform. ViSP is a complete cross-platform\rlibrary that allows prototyping and developing applications in visual tracking\rand visual servoing. \r\rThis package contains headers and library necessary for developing software\rthat uses ViSP. \r\rViSP web site address is http://www.irisa.fr/lagadic/visp/visp.html")
