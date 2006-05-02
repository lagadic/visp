##
## Copyright Projet Lagadic / IRISA-INRIA Rennes, 2006
## www: http://www.irisa.fr/lagadic
##
## Author: Fabien Spindler email:Fabien.Spindler@irisa.fr
##
## Try to find pthread library
## Once run this will define: 
##
## PTHREAD_FOUND
## PTHREAD_INCLUDE_DIR
## PTHREAD_LIBRARIES
##

#IF(NOT UNIX)
#  SET(PTHREAD_FOUND FALSE)
#ELSE(NOT UNIX)
  
  FIND_PATH(PTHREAD_INCLUDE_DIR pthread.h
    /usr/include
    $ENV{PTHREAD_INCLUDE_PATH}
    )
  #MESSAGE("DBG PTHREAD_INCLUDE_DIR=${PTHREAD_INCLUDE_DIR}")  
  
  # pthreadVSE pthreadGCE pthreadGC pthreadVC1 pthreadVC2 are comming from web
  FIND_LIBRARY(PTHREAD_LIBRARY
    NAMES pthread pthreadVSE pthreadGCE pthreadGC pthreadVC1 pthreadVC2
    PATHS
    /usr/lib
    /usr/local/lib
    /lib    
    $ENV{PTHREAD_LIBRARY_PATH}
    )

  #MESSAGE(STATUS "DBG PTHREAD_LIBRARY=${PTHREAD_LIBRARY}")
  
  ## --------------------------------
  
  IF(PTHREAD_LIBRARY)
    SET(PTHREAD_LIBRARIES ${PTHREAD_LIBRARY})
  ELSE(PTHREAD_LIBRARY)
    #MESSAGE(SEND_ERROR "pthread library not found.")
  ENDIF(PTHREAD_LIBRARY)
  
  IF(NOT PTHREAD_INCLUDE_DIR)
    #MESSAGE(SEND_ERROR "pthread include dir not found.")
  ENDIF(NOT PTHREAD_INCLUDE_DIR)
  
  IF(PTHREAD_LIBRARIES AND PTHREAD_INCLUDE_DIR)
    SET(PTHREAD_FOUND TRUE)
  ELSE(PTHREAD_LIBRARIES AND PTHREAD_INCLUDE_DIR)
    SET(PTHREAD_FOUND FALSE)
  ENDIF(PTHREAD_LIBRARIES AND PTHREAD_INCLUDE_DIR)
  
  #MARK_AS_ADVANCED(
  #  PTHREAD_INCLUDE_DIR
  #  PTHREAD_LIBRARIES
  #  )
  #MESSAGE(STATUS "PTHREAD_FOUND : ${PTHREAD_FOUND}")

#ENDIF(NOT UNIX)
