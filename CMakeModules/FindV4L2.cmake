##
## Copyright Projet Lagadic / IRISA-INRIA Rennes, 2006
## www: http://www.irisa.fr/lagadic
##
## Author: Fabien Spindler email:Fabien.Spindler@irisa.fr
##
## Try to find linux/videodev.h for Video For Linux Two framegrabbing 
## capabilities
## Once run this will define: 
##
## V4L_FOUND
## V4L_INCLUDE_DIR
##

IF(NOT UNIX)
  # MESSAGE("FindV4L2.cmake: only available for Unix.")
  SET(V4L2_FOUND FALSE)
ELSE(NOT UNIX)
  
  FIND_PATH(V4L2_INCLUDE_DIR linux/videodev2.h
    $ENV{V4L2_HOME}/include
    /usr/include 
    /usr/src/linux/include)
  #MESSAGE("DBG V4L2_INCLUDE_DIR=${V4L2_INCLUDE_DIR}")  
    
  ## --------------------------------
    
  # IF(NOT V4L2_INCLUDE_DIR)
  #  MESSAGE(SEND_ERROR "Video For Linux Two include dir not found.")
  #ENDIF(NOT V4L2_INCLUDE_DIR)
  
  IF(V4L2_INCLUDE_DIR)
    SET(V4L2_FOUND TRUE)
  ELSE(V4L2_INCLUDE_DIR)
    SET(V4L2_FOUND FALSE)
  ENDIF(V4L2_INCLUDE_DIR)
  
  MARK_AS_ADVANCED(
    V4L2_INCLUDE_DIR
    )
ENDIF(NOT UNIX)
