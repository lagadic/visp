##
## Copyright Projet Lagadic / IRISA-INRIA Rennes, 2006
## www: http://www.irisa.fr/lagadic
##
## Author: Fabien Spindler email:Fabien.Spindler@irisa.fr
##
## This module looks for Doxygen executable and the dot executable which could 
## be used to generate html and graphical documentation from source code. 
## 
## It will set the following variables:
##
##  DOXYGEN_FOUND
##  DOXYGEN_EXECUTABLE
##
##  DOXYGEN_DOT_FOUND
##  DOXYGEN_DOT_EXECUTABLE
##  DOXYGEN_DOT_EXECUTABLE_PATH
##
## see http://www.doxygen.org


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
