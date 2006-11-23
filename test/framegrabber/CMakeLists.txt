#############################################################################
#
# $Id: CMakeLists.txt,v 1.6 2006-11-23 17:29:07 fspindle Exp $
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
  test1394.cpp
  test1394Two.cpp
  testDisk.cpp
  testIcComp.cpp
  testItifg8.cpp
  testItifg8Multi.cpp
  testOSXcfoxColor.cpp
  testV4l2Color.cpp
  testV4l2Grey.cpp
  testDirectShow.cpp
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
ADD_TEST(test1394         test1394 -d)
ADD_TEST(testDisk         testDisk -d)
ADD_TEST(testIcComp       testIcComp -d)
ADD_TEST(testItifg8       testItifg8 -d)
ADD_TEST(testItifg8Multi  testItifg8Multi -d)
ADD_TEST(testOSXcfoxColor testOSXcfoxColor -d)
ADD_TEST(testV4l2Color    testV4l2Color -d)
ADD_TEST(testV4l2Grey     testV4l2Grey -d)
ADD_TEST(testDirectShow   testDirectShow -d)

# customize clean target 
SET_DIRECTORY_PROPERTIES(PROPERTIES 
  ADDITIONAL_MAKE_CLEAN_FILES "core*;*~;gmon.out;DartTestfile.txt"
)
