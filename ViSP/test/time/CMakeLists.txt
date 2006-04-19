#
# Copyright Projet Lagadic / IRISA-INRIA Rennes, 2006
# www: http://www.irisa.fr/lagadic
#
# Author: Fabien Spindler email:Fabien.Spindler@irisa.fr
#
# SOURCE variable corresponds to the list of all the sources to build binaries.
# The generate binary comes by removing the .cpp extension to
# the source name.
#
# If you want to add/remove a source, modify here
SET (SOURCE
  testTime.cpp
)

# rule for binary build
FOREACH(source ${SOURCE})
  # Compute the name of the binary to create
  GET_FILENAME_COMPONENT(binary ${source} NAME_WE)

  # From source compile the binary and add link rules
  ADD_EXECUTABLE(${binary} ${source})
  TARGET_LINK_LIBRARIES(${binary} ${VISP_INTERN_LIBS} ${VISP_EXTERN_LIBS})

  # Add test
  ADD_TEST(${binary} ${binary})

ENDFOREACH(source)

# customize clean target 
SET_DIRECTORY_PROPERTIES(PROPERTIES 
  ADDITIONAL_MAKE_CLEAN_FILES "core*;*~;gmon.out;DartTestfile.txt"
)
