PROJECT(OgreTutorial)

FIND_PACKAGE(VISP REQUIRED)
IF(VISP_FOUND)
  INCLUDE(${VISP_USE_FILE})
ENDIF(VISP_FOUND)

ADD_EXECUTABLE(HelloWorldOgre HelloWorldOgre.cpp)
ADD_EXECUTABLE(HelloWorldOgreAdvanced HelloWorldOgreAdvanced.cpp)

