PROJECT(ImageManipulation)

FIND_PACKAGE(VISP REQUIRED)
IF(VISP_FOUND)
  INCLUDE(${VISP_USE_FILE})
ENDIF(VISP_FOUND)

ADD_EXECUTABLE(display display.cpp)
ADD_EXECUTABLE(grab1394-1 grab1394-1.cpp)
ADD_EXECUTABLE(grab1394-2 grab1394-2.cpp)
ADD_EXECUTABLE(grabDirectShow grabDirectShow.cpp)
ADD_EXECUTABLE(grabDisk grabDisk.cpp)
ADD_EXECUTABLE(grabOSXcfox grabOSXcfox.cpp)
ADD_EXECUTABLE(grabV4l2 grabV4l2.cpp)
