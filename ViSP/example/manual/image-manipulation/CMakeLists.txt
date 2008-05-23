PROJECT(ImageManipulation)

FIND_PACKAGE(VISP REQUIRED)
IF(VISP_FOUND)
  INCLUDE(${VISP_USE_FILE})
ENDIF(VISP_FOUND)

ADD_EXECUTABLE(manDisplay        manDisplay.cpp)
ADD_EXECUTABLE(manGrab1394-1     manGrab1394-1.cpp)
ADD_EXECUTABLE(manGrab1394-2     manGrab1394-2.cpp)
ADD_EXECUTABLE(manGrabDirectShow manGrabDirectShow.cpp)
ADD_EXECUTABLE(manGrabDisk       manGrabDisk.cpp)
ADD_EXECUTABLE(manGrabOSXcfox    manGrabOSXcfox.cpp)
ADD_EXECUTABLE(manGrabV4l2       manGrabV4l2.cpp)

