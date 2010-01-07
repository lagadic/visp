#############################################################################
#
# $Id$
#
# Copyright (C) 1998-2010 Inria. All rights reserved.
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
