#############################################################################
#
# $Id: CMakeLists.txt,v 1.3 2007-01-24 16:21:18 asaunier Exp $
#
# Copyright (C) 1998-2006 Inria. All rights reserved.
#
# This software was developed at:
# IRISA/INRIA Rennes
# Projet Lagadic
# Campus Universitaire de Beaulieu
# 35042 Rennes Cedex
# http://www.irisa.fr/lagadic
#
# This file is part of the ViSP toolkit
#
# This file may be distributed under the terms of the Q Public License
# as defined by Trolltech AS of Norway and appearing in the file
# LICENSE included in the packaging of this file.
#
# Licensees holding valid ViSP Professional Edition licenses may
# use this file in accordance with the ViSP Commercial License
# Agreement provided with the Software.
#
# This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
# WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
#
# Contact visp@irisa.fr if any conditions of this licensing are
# not clear to you.
#
# Description:
# ViSP overall configuration file. 
#
# Authors:
# Fabien Spindler
#
#############################################################################

# Set SRC_SUBDIRS variable to all the sub directories we want to parse during
# the build process. 
#
# If you add/remove a directory, modify here
SET (SRC_SUBDIRS
  feature
  homography
  image
  math
  pose
  servo-afma4
  servo-afma6
  servo-biclops
  servo-ptu46
  servo-simu
  simulator
  time
  tracking
)

# Build process propagation in the sub directories
SUBDIRS(${SRC_SUBDIRS})

# customize clean target 
SET_DIRECTORY_PROPERTIES(PROPERTIES 
  ADDITIONAL_MAKE_CLEAN_FILES "core*;*~;gmon.out;DartTestfile.txt"
)
