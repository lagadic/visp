#############################################################################
#
# $Id: CMakeLists.txt,v 1.3 2006-09-08 16:27:58 fspindle Exp $
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
  test1ServoPoint.cpp
  test2Servo2D0.5.cpp
  test2ServoPoint.cpp
  test3ServoPoint.cpp
  testServo2D0.5.cpp
  testServo3D.cpp
  testServo3dPoint.cpp
  testServo4Points.cpp
  testServo4PointsDisplay.cpp
  testServoCircle1.cpp
  testServoCircle2.cpp
  testServoCylinder1.cpp
  testServoLine1.cpp
  testServoSphere1.cpp
  testServoSphereSecondaryTask.cpp
  testServoThetaU.cpp
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
ADD_TEST(test1ServoPoint              test1ServoPoint)
ADD_TEST(test2Servo2D0.5              test2Servo2D0.5)
ADD_TEST(test2ServoPoint              test2ServoPoint)
ADD_TEST(test3ServoPoint              test3ServoPoint)
ADD_TEST(testServo2D0.5               testServo2D0.5)
ADD_TEST(testServo3D                  testServo3D)
ADD_TEST(testServo3dPoint             testServo3dPoint)
ADD_TEST(testServo4Points             testServo4Points)
ADD_TEST(testServo4PointsDisplay      testServo4PointsDisplay -d)
ADD_TEST(testServoCircle1             testServoCircle1)
ADD_TEST(testServoCircle2             testServoCircle2)
ADD_TEST(testServoCylinder1           testServoCylinder1)
ADD_TEST(testServoLine1               testServoLine1)
ADD_TEST(testServoSphere1             testServoSphere1)
ADD_TEST(testServoSphereSecondaryTask testServoSphereSecondaryTask)
ADD_TEST(testServoThetaU              testServoThetaU)

# customize clean target 
SET_DIRECTORY_PROPERTIES(PROPERTIES 
  ADDITIONAL_MAKE_CLEAN_FILES "core*;*~;gmon.out;DartTestfile.txt"
)
