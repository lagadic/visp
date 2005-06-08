dnl Calls IRISA_HAVE_IEEE1394_FG (contained in this file) as a subroutine.
AC_DEFUN([IRISA_HAVE_IEEE1394_FG],
[

#-----------------------------------------------------------------------
# Manage the --with-fg-ieee1394-dir option
AC_ARG_WITH([fg-ieee1394-dir],
  AC_HELP_STRING([--with-fg-ieee1394-dir=DIR],
		 [Location of Firewire IEEE1394 framegrabber api. By default /udd/fspindle/robot/IEEE1394/current]),
                 path_fg_ieee1394=$withval, 
                 path_fg_ieee1394=/udd/fspindle/robot/IEEE1394/current)
# check if the firewire Ieee1394 framegrabber home dir exists
#AC_CHECK_FILE([$path_fg_ieee1394], [with_fg_ieee1394=yes], [with_fg_ieee1394=no])
with_fg_ieee1394=yes

# check if libraw1394 is reachable
AC_MSG_CHECKING(whether libraw1394 works)
AC_PREPROC_IFELSE([AC_LANG_SOURCE([[#include <libraw1394/raw1394.h>]])],[have_include_raw1394=yes],[have_include_raw1394=no])

if test "x$have_include_raw1394" = "xyes"; then
  AC_MSG_RESULT(yes)
  AC_DEFINE(HAVE_INCLUDE_RAW1394, 1, 
    [Define to 1 if the include libraw1394/raw1394.h for firewire acquisition is available])

  # check if libraw1394 is reachable
  AC_MSG_CHECKING(whether libdc1394 works)
  AC_PREPROC_IFELSE([AC_LANG_SOURCE([[#include <libdc1394/dc1394_control.h>]])],[have_include_dc1394=yes],[have_include_dc1394=no])
  if test "x$have_include_dc1394" = "xyes"; then
    AC_MSG_RESULT(yes)
    AC_DEFINE(HAVE_INCLUDE_DC1394, 1, 
      [Define to 1 if the include libdc1394/dc1394_control.h for firewire acquisition is available])


    # set some variables if the firewire framegrabber is reacheable
    AC_MSG_CHECKING(whether framegrabber Ieee1394 api (firewire) works)
    # set some variables if the firewire framegrabber is reacheable
    if test "x$with_fg_ieee1394" = "xyes"; then
      FG_IEEE1394_HOME=$path_fg_ieee1394
      FG_IEEE1394_DIR_INCLUDE=$FG_IEEE1394_HOME/include
      FG_IEEE1394_DIR_LIB=$FG_IEEE1394_HOME/lib/$OS/$CXX_VERSION
      FG_IEEE1394_CXXFLAGS=-I$FG_IEEE1394_DIR_INCLUDE
      FG_IEEE1394_LDFLAGS="-L$FG_IEEE1394_DIR_LIB"
      FG_IEEE1394_LIBS="-lieee1394 -lraw1394 -ldc1394_control"

      # save values
      TMP_CXXFLAGS=$CXXFLAGS
      TMP_CPPFLAGS=$CPPFLAGS
      TMP_LDFLAGS=$LDFLAGS
      TMP_LIBS=$LIBS
      # set linker options for a local test 
      CXXFLAGS="$FG_IEEE1394_CXXFLAGS -I$VISP_DIR_INCLUDE"
      LDFLAGS=$FG_IEEE1394_LDFLAGS
      LIBS=$FG_IEEE1394_LIBS
      # test a single function of the firewire api
      AC_LANG_PUSH(C++)		
      AC_LINK_IFELSE(
        [AC_LANG_PROGRAM([#include "CIEEE1394.h"],
                         [CIEEE1394 fg;])],
        have_lib_ieee1394=yes,
        have_lib_ieee1394=no)
      AC_LANG_POP(C++)		
      # put the saved values
      CXXFLAGS=$TMP_CXXFLAGS
      CPPFLAGS=$TMP_CPPFLAGS
      LDFLAGS=$TMP_LDFLAGS
      LIBS=$TMP_LIBS

      if test "x$have_lib_ieee1394" = "xyes"; then
        # firewire api works
        CXXFLAGS="$CXXFLAGS $FG_IEEE1394_CXXFLAGS"
        LDFLAGS="$LDFLAGS $FG_IEEE1394_LDFLAGS"
        LIBS="$LIBS $FG_IEEE1394_LIBS"
        AC_DEFINE([HAVE_FG_IEEE1394], 1,
                [Define to 1 if the Ieee1394 (firewire) framegrabber api is available])
        AC_MSG_RESULT(yes)
        AC_SUBST(FG_IEEE1394_CXXFLAGS)
        AC_SUBST(FG_IEEE1394_LDFLAGS)
        AC_SUBST(FG_IEEE1394_LIBS)
      else
        AC_MSG_RESULT(no)
      fi
    else
      AC_MSG_RESULT(no)
    fi # test for libieee1394

  else # test for libdc1394
    AC_MSG_RESULT(no)
  fi
else # test for libraw1394
  AC_MSG_RESULT(no)
fi


])