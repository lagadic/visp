##
## Copyright Projet Lagadic / IRISA-INRIA Rennes, 2006
## www: http://www.irisa.fr/lagadic
##
## Author: Fabien Spindler email:Fabien.Spindler@irisa.fr
##
## Try to find cfox library for Apple OS X
## Once run this will define: 
##
## CFOX_FOUND
## CFOX_INCLUDE_DIR
## CFOX_LIBRARIES
##

IF(NOT APPLE)
  SET(CFOX_FOUND FALSE)
ELSE(NOT APPLE)
  
  FIND_PATH(CFOX_INCLUDE_DIR cfox.h
    /sw/include/cfox
    )
  #MESSAGE(STATUS "DBG CFOX_INCLUDE_DIR=${CFOX_INCLUDE_DIR}")  
  
  FIND_LIBRARY(CFOX_LIBRARY
    NAMES cfox
    PATHS
    /sw/lib
    )

  #MESSAGE(STATUS "DBG CFOX_LIBRARY=${CFOX_LIBRARY}")
  
  ## --------------------------------
  
  IF(CFOX_LIBRARY)
    SET(CFOX_LIBRARIES ${CFOX_LIBRARY})
  ELSE(CFOX_LIBRARY)
    MESSAGE(SEND_ERROR "cfox library not found.")
  ENDIF(CFOX_LIBRARY)
  
  IF(NOT CFOX_INCLUDE_DIR)
    MESSAGE(SEND_ERROR "cfox include dir not found.")
  ENDIF(NOT CFOX_INCLUDE_DIR)
  
  IF(CFOX_LIBRARIES AND CFOX_INCLUDE_DIR)
    SET(CFOX_FOUND TRUE)
  ELSE(CFOX_LIBRARIES AND CFOX_INCLUDE_DIR)
    SET(CFOX_FOUND FALSE)
  ENDIF(CFOX_LIBRARIES AND CFOX_INCLUDE_DIR)
  
  MARK_AS_ADVANCED(
    CFOX_INCLUDE_DIR
    CFOX_LIBRARIES
    )
  #MESSAGE(STATUS "CFOX_FOUND : ${CFOX_FOUND}")

ENDIF(NOT APPLE)
