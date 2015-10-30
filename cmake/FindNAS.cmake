#############################################################################
#
# This file is part of the ViSP software.
# Copyright (C) 2005 - 2015 by Inria. All rights reserved.
#
# This software is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# ("GPL") version 2 as published by the Free Software Foundation.
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
# Try to determine if Inria's NAS server hosting /udd/ is available 
#
# NAS_FOUND
#
# Authors:
# Fabien Spindler
#
#############################################################################

    
FIND_FILE(AFMA6_CONSTANT_FILE_ON_NAS
  NAMES const_Afma6.cnf
  PATHS 
  /udd/fspindle/robot/Afma6/current/include
  Z:/robot/Afma6/current/include
  )

#MESSAGE("DBG AFMA6_CONSTANT_FILE_ON_NAS=${AFMA6_CONSTANT_FILE_ON_NAS}")
 
## --------------------------------
  
IF(AFMA6_CONSTANT_FILE_ON_NAS)
  SET(NAS_FOUND TRUE)
ELSE(AFMA6_CONSTANT_FILE_ON_NAS)
  SET(NAS_FOUND FALSE)
ENDIF(AFMA6_CONSTANT_FILE_ON_NAS)

MARK_AS_ADVANCED(
  AFMA6_CONSTANT_FILE_ON_NAS
)
