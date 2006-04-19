##
## Copyright Projet Lagadic / IRISA-INRIA Rennes, 2006
## www: http://www.irisa.fr/lagadic
##
## Author: Fabien Spindler email:Fabien.Spindler@irisa.fr
##
## add custom target distclean
## cleans and removes cmake generated files etc.
##

ADD_CUSTOM_TARGET( distclean  DEPENDS clean)

SET(DISTCLEAN_FILES
  cmake.check_cache
  */cmake.check_cache
  cmake_install.cmake
  */cmake_install.cmake
  */*/cmake_install.cmake
  CMakeCache.txt
  core core.*
  gmon
  *~ 
  *%
  SunWS_cache
  ii_files
  )

## for 1.8.x:
ADD_CUSTOM_COMMAND(
  TARGET distclean
  COMMAND 
  ${CMAKE_COMMAND} -E remove ${DISTCLEAN_FILES}
  COMMENT
  )
