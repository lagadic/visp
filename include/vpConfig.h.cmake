#ifndef vpConfig_h
#define vpConfig_h

// Defined if X11 library available.
#cmakedefine VISP_HAVE_X11

// Defined if GTK library available (either gtk or gtk2).
#cmakedefine VISP_HAVE_GTK

// Defined if GSL library available (-lgsl -lgslcblas).
#cmakedefine VISP_HAVE_GSL

// Defined if Coin library available.
#cmakedefine VISP_HAVE_COIN

// Defined if Qt library available (either Qt-3 or Qt-4).
#cmakedefine VISP_HAVE_QT

// Defined if SoQt library available.
#cmakedefine VISP_HAVE_SOQT

// Defined if dc1394_control and raw1394 libraries available.
#cmakedefine VISP_HAVE_DC1394

// Defined if cfox library is available (only under MAC OS X).
#cmakedefine VISP_HAVE_CFOX

// Defined if Video For Linux Two available.
#cmakedefine VISP_HAVE_V4L2

// Defined if Irisa's ICcomp framegraber available.
#cmakedefine VISP_HAVE_ICCOMP

// Defined if Irisa's Afma4 robot available.
#cmakedefine VISP_HAVE_AFMA4

// Defined if Irisa's Afma6 robot available.
#cmakedefine VISP_HAVE_AFMA6

// Defined if Biclops pan-tilt head available.
#cmakedefine VISP_HAVE_BICLOPS

// Defined if Irisa's Ptu-46 pan-tilt head available.
#cmakedefine VISP_HAVE_PTU46


#endif
