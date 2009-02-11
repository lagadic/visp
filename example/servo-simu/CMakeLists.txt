#############################################################################
#
# $Id: CMakeLists.txt,v 1.3 2008-10-31 17:48:29 fspindle Exp $
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
servoSimu3DCamVelocity.cpp
servoSimu3DCamVelocity2.cpp
servoSimuCircle2DCamVelocity.cpp
servoSimuCircle2DCamVelocityDisplay.cpp
servoSimuCylinder2DCamVelocityDisplay.cpp
servoSimuCylinder2DCamVelocityDisplaySecondaryTask.cpp
servoSimuFourPoints2DCamVelocity.cpp
servoSimuFourPoints2DCamVelocityDisplay.cpp
servoSimuFourPoints2DPolarCamVelocityDisplay.cpp
servoSimuLine2DCamVelocityDisplay.cpp
servoSimuPoint2DCamVelocity1.cpp
servoSimuPoint2DCamVelocity2.cpp
servoSimuPoint2DCamVelocity3.cpp
servoSimuPoint2DhalfCamVelocity1.cpp
servoSimuPoint2DhalfCamVelocity2.cpp
servoSimuPoint2DhalfCamVelocity3.cpp
servoSimuPoint3DCamVelocity.cpp
servoSimuSphere2DCamVelocity.cpp
servoSimuSphere2DCamVelocitySecondaryTask.cpp
servoSimuSquareLine2DCamVelocityDisplay.cpp
servoSimuThetaUCamVelocity.cpp
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
ADD_TEST(servoSimu3DCamVelocity				servoSimu3DCamVelocity				)	
ADD_TEST(servoSimuCircle2DCamVelocity			servoSimuCircle2DCamVelocity			)
ADD_TEST(servoSimuCircle2DCamVelocityDisplay		servoSimuCircle2DCamVelocityDisplay	-d	)
ADD_TEST(servoSimuCylinder2DCamVelocityDisplay		servoSimuCylinder2DCamVelocityDisplay	-d	)
ADD_TEST(servoSimuFourPoints2DCamVelocity		servoSimuFourPoints2DCamVelocity		)
ADD_TEST(servoSimuFourPoints2DCamVelocityDisplay	servoSimuFourPoints2DCamVelocityDisplay -d	)
ADD_TEST(servoSimuLine2DCamVelocityDisplay		servoSimuLine2DCamVelocityDisplay	-d	)
ADD_TEST(servoSimuPoint2DCamVelocity1			servoSimuPoint2DCamVelocity1			)
ADD_TEST(servoSimuPoint2DCamVelocity2			servoSimuPoint2DCamVelocity2			)
ADD_TEST(servoSimuPoint2DCamVelocity3			servoSimuPoint2DCamVelocity3			)
ADD_TEST(servoSimuPoint2DhalfCamVelocity1		servoSimuPoint2DhalfCamVelocity1		)
ADD_TEST(servoSimuPoint2DhalfCamVelocity2		servoSimuPoint2DhalfCamVelocity2		)
ADD_TEST(servoSimuPoint2DhalfCamVelocity3		servoSimuPoint2DhalfCamVelocity3		)
ADD_TEST(servoSimuPoint3DCamVelocity			servoSimuPoint3DCamVelocity			)
ADD_TEST(servoSimuSphere2DCamVelocity			servoSimuSphere2DCamVelocity			)
ADD_TEST(servoSimuSphere2DCamVelocitySecondaryTask	servoSimuSphere2DCamVelocitySecondaryTask	)
ADD_TEST(servoSimuThetaUCamVelocity			servoSimuThetaUCamVelocity			)

# customize clean target 
SET_DIRECTORY_PROPERTIES(PROPERTIES 
  ADDITIONAL_MAKE_CLEAN_FILES "core*;*~;gmon.out;DartTestfile.txt"
)
