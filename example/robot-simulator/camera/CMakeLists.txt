#############################################################################
#
# $Id: CMakeLists.txt 2457 2010-01-07 10:41:18Z nmelchio $
#
# This file is part of the ViSP software.
# Copyright (C) 2005 - 2014 by INRIA. All rights reserved.
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
servoSimu3D_cdMc_CamVelocity.cpp
servoSimu3D_cdMc_CamVelocityWithoutVpServo.cpp
servoSimu3D_cMcd_CamVelocity.cpp
servoSimu3D_cMcd_CamVelocityWithoutVpServo.cpp
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
  TARGET_LINK_LIBRARIES(${binary} ${VISP_INTERN_LIBRARY} ${VISP_EXTERN_LIBRARIES})

ENDFOREACH(source)

# Add test
ADD_TEST(servoSimu3D_cdMc_CamVelocity			servoSimu3D_cdMc_CamVelocity			)	
ADD_TEST(servoSimu3D_cdMc_CamVelocityWithoutVpServo	servoSimu3D_cdMc_CamVelocityWithoutVpServo	)	
ADD_TEST(servoSimu3D_cMcd_CamVelocity			servoSimu3D_cMcd_CamVelocity			)	
ADD_TEST(servoSimu3D_cMcd_CamVelocityWithoutVpServo	servoSimu3D_cMcd_CamVelocityWithoutVpServo	)	
ADD_TEST(servoSimuCircle2DCamVelocity			servoSimuCircle2DCamVelocity			)
ADD_TEST(servoSimuCircle2DCamVelocityDisplay		servoSimuCircle2DCamVelocityDisplay	-c ${OPTION_TO_DESACTIVE_DISPLAY}	)
ADD_TEST(servoSimuCylinder2DCamVelocityDisplay		servoSimuCylinder2DCamVelocityDisplay	-c ${OPTION_TO_DESACTIVE_DISPLAY}	)
ADD_TEST(servoSimuFourPoints2DCamVelocity		servoSimuFourPoints2DCamVelocity		)
ADD_TEST(servoSimuFourPoints2DCamVelocityDisplay	servoSimuFourPoints2DCamVelocityDisplay -c ${OPTION_TO_DESACTIVE_DISPLAY}	)
ADD_TEST(servoSimuLine2DCamVelocityDisplay		servoSimuLine2DCamVelocityDisplay	-c ${OPTION_TO_DESACTIVE_DISPLAY}	)
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
