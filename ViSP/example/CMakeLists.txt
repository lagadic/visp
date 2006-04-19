#
# Copyright Projet Lagadic / IRISA-INRIA Rennes, 2006
# www: http://www.irisa.fr/lagadic
#
# Author: Fabien Spindler email:Fabien.Spindler@irisa.fr
#

# Set SRC_SUBDIRS variable to all the sub directories we want to parse during
# the build process. 
#
# If you add/remove a directory, modify here
SET (SRC_SUBDIRS
  image
  pose-estimation
  simulation
  tracking
)

# Build process propagation in the sub directories
SUBDIRS(${SRC_SUBDIRS})

# customize clean target 
SET_DIRECTORY_PROPERTIES(PROPERTIES 
  ADDITIONAL_MAKE_CLEAN_FILES "core*;*~;gmon.out;DartTestfile.txt"
)
