#############################################################################
#
# $Id: FindDoxygen.cmake,v 1.2 2006-05-30 08:35:00 fspindle Exp $
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
# This module looks for Doxygen executable and the dot executable which could 
# be used to generate html and graphical documentation from source code. 
# 
# It will set the following variables:
#
#  DOXYGEN_FOUND
#  DOXYGEN_EXECUTABLE
#
#  DOXYGEN_DOT_FOUND
#  DOXYGEN_DOT_EXECUTABLE
#  DOXYGEN_DOT_EXECUTABLE_PATH
#
# See http://www.doxygen.org
#
# Authors:
# Fabien Spindler
#
#############################################################################

FIND_PROGRAM(DOXYGEN_EXECUTABLE
  NAMES doxygen
  DOC "where is the doxygen executable?"
  PATHS 
  $ENV{DOXYGEN_HOME}
  "[HKEY_LOCAL_MACHINE\\SOFTWARE\\Microsoft\\Windows\\CurrentVersion\\Uninstall\\doxygen_is1;Inno Setup: App Path]/bin"
  "C:/Program Files/doxygen/bin"
  "C:/Programme/doxygen/bin"
  )
  #MESSAGE("DBG DOXYGEN_EXECUTABLE=${DOXYGEN_EXECUTABLE}")

FIND_PROGRAM(DOXYGEN_DOT_EXECUTABLE
  NAMES dot
  DOC "where is dot executable from Graphiz (for doxygen)?"
  PATHS  
  $ENV{DOT_HOME}
  "[HKEY_LOCAL_MACHINE\\SOFTWARE\\ATT\\Graphviz;InstallPath]/bin"
  "C:/Program Files/ATT/Graphviz/bin"
  "C:/Programme/ATT/Graphviz/bin"
  )
  #MESSAGE("DBG DOXYGEN_DOT_EXECUTABLE=${DOXYGEN_DOT_EXECUTABLE}")


## use variable names conistent with Modules/readme.txt (JW)
IF (DOXYGEN_EXECUTABLE)
  SET (DOXYGEN_FOUND TRUE)

  # for backward compatibility (deprecated)
  SET (DOXYGEN ${DOXYGEN_EXECUTABLE})
ENDIF (DOXYGEN_EXECUTABLE)

IF (DOXYGEN_DOT_EXECUTABLE)
  SET (DOXYGEN_DOT_FOUND TRUE)  
  # the directory of dot is required in doxygen.config: DOT_PATH
  GET_FILENAME_COMPONENT (DOXYGEN_DOT_EXECUTABLE_PATH ${DOXYGEN_DOT_EXECUTABLE} PATH)

  # for backward compatibility (deprecated)
  SET (DOT ${DOXYGEN_DOT_EXECUTABLE})
ENDIF (DOXYGEN_DOT_EXECUTABLE)




MARK_AS_ADVANCED(
  DOXYGEN_EXECUTABLE
  DOXYGEN_DOT_EXECUTABLE
  DPXYGEN_DOT_EXECUTABLE_DIR
  DOXYGEN
  DOT
  )
