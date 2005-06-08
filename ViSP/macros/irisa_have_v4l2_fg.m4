dnl Calls IRISA_HAVE_V4L2_FG (contained in this file) as a subroutine.
AC_DEFUN([IRISA_HAVE_V4L2_FG],
[
#-----------------------------------------------------------------------
# Manage the --with-fg-v4l2-dir option
AC_ARG_WITH([fg-v4l2-dir],
  AC_HELP_STRING([--with-fg-v4l2-dir=DIR],
		 [Location of Video4Linux2 framegrabber api. By default /udd/fspindle/robot/V4L2/current]),
                 path_fg_v4l2=$withval, 
                 path_fg_v4l2=/udd/fspindle/robot/V4L2/current)
# check if the Video4Linux2 framegrabber home dir exists
#AC_CHECK_FILE([$path_fg_v4l2], [with_fg_v4l2=yes], [with_fg_v4l2=no])
with_fg_v4l2=yes

# check if libraw1394 is reachable
AC_MSG_CHECKING(whether Video For Linux Two api works)
AC_PREPROC_IFELSE([AC_LANG_SOURCE([[#include </usr/src/linux/include/linux/videodev2.h>]])],[have_include_videodev2=yes],[have_include_videodev2=no])

if test "x$have_include_videodev2" = "xyes"; then
  AC_DEFINE(HAVE_INCLUDE_VIDEODEV2, 1, 
    [Define to 1 if the include linux/videodev2.h for Video4Linux2 acquisition is available])

  # set some variables if the V4L2 framegrabber api is reacheable
  FG_V4L2_HOME=$path_fg_v4l2
  FG_V4L2_DIR_INCLUDE=$FG_V4L2_HOME/include
  FG_V4L2_DIR_LIB=$FG_V4L2_HOME/lib/$OS/$CXX_VERSION
  FG_V4L2_CXXFLAGS="-I$FG_V4L2_DIR_INCLUDE -I/usr/src/linux/include"
  FG_V4L2_LDFLAGS="-L$FG_V4L2_DIR_LIB"
  FG_V4L2_LIBS="-lv4l2-0.9.x"
  # save values
  TMP_CXXFLAGS=$CXXFLAGS
  TMP_CPPFLAGS=$CPPFLAGS
  TMP_LDFLAGS=$LDFLAGS
  TMP_LIBS=$LIBS
  # set linker options for a local test 
  CXXFLAGS=$FG_V4L2_CXXFLAGS
  LDFLAGS=$FG_V4L2_LDFLAGS
  LIBS=$FG_V4L2_LIBS
  # test a single function of the V4L2 api
  AC_LANG_PUSH(C++)		
  AC_LINK_IFELSE(
    [AC_LANG_PROGRAM([#include "CBTTV09x.h"],
                     [CBTTV09x fg;])],
    have_lib_v4l2=yes,
    have_lib_v4l2=no)
  AC_LANG_POP(C++)		
  # put the saved values
  CXXFLAGS=$TMP_CXXFLAGS
  CPPFLAGS=$TMP_CPPFLAGS
  LDFLAGS=$TMP_LDFLAGS
  LIBS=$TMP_LIBS

  if test "x$have_lib_v4l2" = "xyes"; then
    # firewire api works
    CXXFLAGS="$CXXFLAGS $FG_V4L2_CXXFLAGS"
    LDFLAGS="$LDFLAGS $FG_V4L2_LDFLAGS"
    LIBS="$LIBS $FG_V4L2_LIBS"
    AC_DEFINE([HAVE_FG_V4L2], 1,
      [Define to 1 if the Video4Linux2 framegrabber api is available])
    AC_MSG_RESULT(yes)
    AC_SUBST(FG_V4L2_CXXFLAGS)
    AC_SUBST(FG_V4L2_LDFLAGS)
    AC_SUBST(FG_V4L2_LIBS)
  else
    AC_MSG_RESULT(no)
  fi
else
  AC_MSG_RESULT(no)
fi


])