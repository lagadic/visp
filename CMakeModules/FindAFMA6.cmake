##
## Copyright Projet Lagadic / IRISA-INRIA Rennes, 2006
## www: http://www.irisa.fr/lagadic
##
## Author: Fabien Spindler email:Fabien.Spindler@irisa.fr
##
## Try to find libraries for Irisa's Afma 6 ddl cartesian robot 
## Once run this will define: 
##
## AFMA6_FOUND
## AFMA6_INCLUDE_DIR
## AFMA6_LIBRARIES
##

IF(NOT UNIX)
  # MESSAGE("FindAFMA6.cmake: Afma6 only available for Unix.")
  SET(AFMA6_FOUND FALSE)
ELSE(NOT UNIX)
  
  FIND_PATH(AFMA6_INCLUDE_DIR main_Afma6.h
    $ENV{AFMA6_HOME}/include
    /udd/fspindle/robot/Afma6/current/include
    /local/soft/Afma6/current/include 
    )
  #MESSAGE("DBG AFMA6_INCLUDE_DIR=${AFMA6_INCLUDE_DIR}")  
  
  FIND_LIBRARY(ROBOTAFMA6_LIBRARY
    NAMES robotAfma6
    PATHS 
    $ENV{AFMA6_HOME}/lib
    /udd/fspindle/robot/Afma6/current/lib
    /local/soft/Afma6/current/lib
    )
  FIND_LIBRARY(TOOLSAFMA6_LIBRARY
    NAMES toolsAfma6
    PATHS 
    $ENV{AFMA6_HOME}/lib
    /udd/fspindle/robot/Afma6/current/lib
    /local/soft/Afma6/current/lib
    )
  FIND_LIBRARY(UPRIMAFMA6_LIBRARY
    NAMES uprimAfma6
    PATHS 
    $ENV{AFMA6_HOME}/lib
    /udd/fspindle/robot/Afma6/current/lib
    /local/soft/Afma6/current/lib
    )
  FIND_LIBRARY(BIT3AFMA6_LIBRARY
    NAMES bit3Afma6
    PATHS 
    $ENV{AFMA6_HOME}/lib
    /udd/fspindle/robot/Afma6/current/lib
    /local/soft/Afma6/current/lib
    )
  #MESSAGE("DBG AFMA6_LIBRARY=${AFMA6_LIBRARY}")
  
  ## --------------------------------
  
  IF(ROBOTAFMA6_LIBRARY AND TOOLSAFMA6_LIBRARY AND UPRIMAFMA6_LIBRARY 
      AND BIT3AFMA6_LIBRARY)
    SET(AFMA6_LIBRARIES ${ROBOTAFMA6_LIBRARY} ${TOOLSAFMA6_LIBRARY}
      ${UPRIMAFMA6_LIBRARY} ${BIT3AFMA6_LIBRARY})
  ELSE(ROBOTAFMA6_LIBRARY AND TOOLSAFMA6_LIBRARY AND UPRIMAFMA6_LIBRARY 
      AND BIT3AFMA6_LIBRARY)
    MESSAGE(SEND_ERROR "Afma6 library not found.")
  ENDIF(ROBOTAFMA6_LIBRARY AND TOOLSAFMA6_LIBRARY AND UPRIMAFMA6_LIBRARY 
      AND BIT3AFMA6_LIBRARY)
  
  IF(NOT AFMA6_INCLUDE_DIR)
    MESSAGE(SEND_ERROR "Afma6 include dir not found.")
  ENDIF(NOT AFMA6_INCLUDE_DIR)
  
  IF(AFMA6_LIBRARIES AND AFMA6_INCLUDE_DIR)
    SET(AFMA6_FOUND TRUE)
  ELSE(AFMA6_LIBRARIES AND AFMA6_INCLUDE_DIR)
    SET(AFMA6_FOUND FALSE)
  ENDIF(AFMA6_LIBRARIES AND AFMA6_INCLUDE_DIR)
  
  MARK_AS_ADVANCED(
    AFMA6_INCLUDE_DIR
    AFMA6_LIBRARIES
    AFMA6_LIBRARY
    ROBOTAFMA6_LIBRARY
    TOOLSAFMA6_LIBRARY
    UPRIMAFMA6_LIBRARY
    BIT3AFMA6_LIBRARY
    )
ENDIF(NOT UNIX)
