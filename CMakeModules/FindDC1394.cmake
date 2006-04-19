##
## Copyright Projet Lagadic / IRISA-INRIA Rennes, 2006
## www: http://www.irisa.fr/lagadic
##
## Author: Fabien Spindler email:Fabien.Spindler@irisa.fr
##
## Try to find libDC1394 for IEEE1394 camera 
## Once run this will define: 
##
## DC1394_FOUND
## DC1394_INCLUDE_DIR
## DC1394_LIBRARIES

IF(NOT UNIX)
  # MESSAGE("FindDC1394.cmake: libdc1394 only available for Unix.")
  SET(DC1394_FOUND FALSE)
ELSE(NOT UNIX)
  
  FIND_PATH(DC1394_INCLUDE_DIR libdc1394/dc1394_control.h
    $ENV{DC1394_HOME}/include
    /usr/include )
  #MESSAGE("DBG DC1394_INCLUDE_DIR=${DC1394_INCLUDE_DIR}")  
  
  FIND_LIBRARY(DC1394_LIBRARY
    NAMES dc1394_control
    PATHS 
    $ENV{DC1394_HOME}/lib
    /usr/lib
    )
  #MESSAGE("DBG DC1394_LIBRARY=${DC1394_LIBRARY}")
  
  ## --------------------------------
  
  IF(DC1394_LIBRARY)
    SET(DC1394_LIBRARIES ${DC1394_LIBRARY})
  ELSE(DC1394_LIBRARY)
    #MESSAGE("libdc1394 library not found.")
  ENDIF(DC1394_LIBRARY)
  
  IF(NOT DC1394_INCLUDE_DIR)
    #MESSAGE("libdc1394 include dir not found.")
  ENDIF(NOT DC1394_INCLUDE_DIR)
  
  IF(DC1394_LIBRARIES AND DC1394_INCLUDE_DIR)
    SET(DC1394_FOUND TRUE)
  ELSE(DC1394_LIBRARIES AND DC1394_INCLUDE_DIR)
    SET(DC1394_FOUND FALSE)
  ENDIF(DC1394_LIBRARIES AND DC1394_INCLUDE_DIR)
  
  MARK_AS_ADVANCED(
    DC1394_INCLUDE_DIR
    DC1394_LIBRARIES
    DC1394_LIBRARY
    )
ENDIF(NOT UNIX)
