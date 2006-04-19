##
## Copyright Projet Lagadic / IRISA-INRIA Rennes, 2006
## www: http://www.irisa.fr/lagadic
##
## Author: Fabien Spindler email:Fabien.Spindler@irisa.fr
##
## Try to find Coin library 
## Once run this will define: 
##
## COIN_FOUND
## COIN_LIBRARIES
##

IF(NOT UNIX)
  SET(COIN_FOUND FALSE)
ELSE(NOT UNIX)
  
  FIND_LIBRARY(COIN_LIBRARY
    NAMES Coin
    PATHS
    /usr/lib
    /usr/local/lib
    /lib    
    )

  #MESSAGE(STATUS "DBG COIN_LIBRARY=${COIN_LIBRARY}")
  
  ## --------------------------------
  
  IF(COIN_LIBRARY)
    SET(COIN_LIBRARIES ${COIN_LIBRARY})
    SET(COIN_FOUND TRUE)
  ELSE(COIN_LIBRARY)
    SET(COIN_FOUND FALSE)
    #MESSAGE("Coin library not found.")
  ENDIF(COIN_LIBRARY)
  
  
  MARK_AS_ADVANCED(
    COIN_LIBRARIES
    )
  #MESSAGE(STATUS "COIN_FOUND : ${COIN_FOUND}")

ENDIF(NOT UNIX)
