##
## Copyright Projet Lagadic / IRISA-INRIA Rennes, 2006
## www: http://www.irisa.fr/lagadic
##
## Author: Fabien Spindler email:Fabien.Spindler@irisa.fr
##
## Try to find SoQt library 
## Once run this will define: 
##
## SOQT_FOUND
## SOQT_LIBRARIES
##

IF(NOT UNIX)
  SET(SOQT_FOUND FALSE)
ELSE(NOT UNIX)
  
  FIND_LIBRARY(SOQT_LIBRARY
    NAMES SoQt
    PATHS
    /usr/lib
    /usr/local/lib
    /lib    
    )

  #MESSAGE(STATUS "DBG SOQT_LIBRARY=${SOQT_LIBRARY}")
  
  ## --------------------------------
  
  IF(SOQT_LIBRARY)
    SET(SOQT_LIBRARIES ${SOQT_LIBRARY})
    SET(SOQT_FOUND TRUE)
  ELSE(SOQT_LIBRARY)
    SET(SOQT_FOUND FALSE)
    #MESSAGE("SoQt library not found.")
  ENDIF(SOQT_LIBRARY)
  
  
  MARK_AS_ADVANCED(
    SOQT_LIBRARIES
    )
  #MESSAGE(STATUS "SOQT_FOUND : ${SOQT_FOUND}")

ENDIF(NOT UNIX)
