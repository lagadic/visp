##
## Copyright Projet Lagadic / IRISA-INRIA Rennes, 2006
## www: http://www.irisa.fr/lagadic
##
## Author: Fabien Spindler email:Fabien.Spindler@irisa.fr
##
## Try to find libraries for Irisa's SBS Bit3 vertical bus driver for
## Afma 4 and 6 robots
## Once run this will define: 
##
## BIT3_FOUND
## BIT3_INCLUDE_DIR
## BIT3_LIBRARIES
##

IF(NOT UNIX)
  # MESSAGE("FindBIT3.cmake: only available for Unix.")
  SET(BIT3_FOUND FALSE)
ELSE(NOT UNIX)
  
  FIND_PATH(BIT3_INCLUDE_DIR btapi.h
    $ENV{BIT3_HOME}/include
    /udd/fspindle/robot/driver/bit3-617/1003/current/include 
    )
  #MESSAGE("DBG BIT3_INCLUDE_DIR=${BIT3_INCLUDE_DIR}")  
  
  FIND_LIBRARY(BIT3_LIBRARY
    NAMES btp
    PATHS 
    $ENV{BIT3_HOME}/lib
    /udd/fspindle/robot/driver/bit3-617/1003/current/include 
    )

  #MESSAGE("DBG BIT3_LIBRARY=${BIT3_LIBRARY}")
  
  ## --------------------------------
  
  IF(BIT3_LIBRARY)
    SET(BIT3_LIBRARIES ${BIT3_LIBRARY})
  ELSE(BIT3_LIBRARY)
    MESSAGE(SEND_ERROR "Bit3 library not found.")
  ENDIF(BIT3_LIBRARY)
  
  IF(NOT BIT3_INCLUDE_DIR)
    MESSAGE(SEND_ERROR "Bit3 include dir not found.")
  ENDIF(NOT BIT3_INCLUDE_DIR)
  
  IF(BIT3_LIBRARIES AND BIT3_INCLUDE_DIR)
    SET(BIT3_INCLUDE_DIR ${BIT3_INCLUDE_DIR})
    ADD_DEFINITIONS(-DBT1003) 
    SET(BIT3_FOUND TRUE)
  ELSE(BIT3_LIBRARIES AND BIT3_INCLUDE_DIR)
    SET(BIT3_FOUND FALSE)
  ENDIF(BIT3_LIBRARIES AND BIT3_INCLUDE_DIR)
  
  MARK_AS_ADVANCED(
    BIT3_INCLUDE_DIR
    BIT3_LIBRARIES
    BIT3_LIBRARY
    )
ENDIF(NOT UNIX)
