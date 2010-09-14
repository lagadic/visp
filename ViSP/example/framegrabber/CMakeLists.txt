#############################################################################
#
# $Id$
#
# This file is part of the ViSP software.
# Copyright (C) 2005 - 2010 by INRIA. All rights reserved.
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
# ViSP configuration file. 
#
# Authors:
# Fabien Spindler
#
#############################################################################

# SOURCE variable corresponds to the list of all the sources to build binaries.
# The generate binary comes by removing the .cpp extension to
# the source name.
#
# If you want to add/remove a source, modify here
SET (SOURCE
  grab1394.cpp
  grab1394Two.cpp
  grabDisk.cpp
  grabItifg8.cpp
  grabItifg8Multi.cpp
  grabV4l2Color.cpp
  grabV4l2Grey.cpp
  grabDirectShow.cpp
  grabDirectShowMulti.cpp
  grabOpenCV.cpp
)

# rule for binary build
FOREACH(source ${SOURCE})
  # Compute the name of the binary to create
  GET_FILENAME_COMPONENT(binary ${source} NAME_WE)

  # From source compile the binary and add link rules
  ADD_EXECUTABLE(${binary} ${source})
  TARGET_LINK_LIBRARIES(${binary} ${VISP_INTERN_LIBS} ${VISP_EXTERN_LIBS})

ENDFOREACH(source)

# Add test
# To run these tests automaticaly using the crontab under Unix or the task
# manager under windows, we have to add the -d option to turn off the display
# of acquired images
ADD_TEST(grab1394         grab1394 -d)
ADD_TEST(grab1394Two      grab1394Two -d)
ADD_TEST(grabDisk         grabDisk -d)
ADD_TEST(grabItifg8       grabItifg8 -d)
ADD_TEST(grabItifg8Multi  grabItifg8Multi -d)
ADD_TEST(grabV4l2Color    grabV4l2Color -d)
ADD_TEST(grabV4l2Grey     grabV4l2Grey -d)
ADD_TEST(grabDirectShow   grabDirectShow -d)
ADD_TEST(grabDirectShowMulti   grabDirectShowMulti -d)
ADD_TEST(grabOpenCV       grabOpenCV -d)

# customize clean target 
SET_DIRECTORY_PROPERTIES(PROPERTIES 
  ADDITIONAL_MAKE_CLEAN_FILES "core*;*~;gmon.out;DartTestfile.txt"
)
