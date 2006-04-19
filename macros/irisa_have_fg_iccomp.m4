dnl Calls IRISA_HAVE_FG_ICCOMP (contained in this file) as a subroutine.
AC_DEFUN([IRISA_HAVE_FG_ICCOMP],
[
#-----------------------------------------------------------------------
# Manage the --with-fg-iccomp option
AC_ARG_WITH([fg-iccomp],
  AC_HELP_STRING([--with-fg-iccomp=DIR],
		 [Location of ICcomp framegrabber api. By default /udd/fspindle/robot/IC-comp/current]),
                 path_fg_iccomp=$withval, 
                 path_fg_iccomp=/udd/fspindle/robot/IC-comp/current)

# check if the IC-comp framegrabber home dir exists
#AC_CHECK_FILE([$path_fg_iccomp], [with_fg_iccomp=yes], [with_fg_iccomp=no])
with_fg_iccomp=yes
# set some variables if the IC-comp framegrabber is reacheable
AC_MSG_CHECKING(whether framegrabber ICcomp api works)
if test "x$with_fg_iccomp" = "xyes"; then
  FG_ICCOMP_HOME=$path_fg_iccomp
  FG_ICCOMP_DIR_INCLUDE=$FG_ICCOMP_HOME/include
  FG_ICCOMP_DIR_LIB=$FG_ICCOMP_HOME/lib
  FG_ICCOMP_CXXFLAGS=-I$FG_ICCOMP_DIR_INCLUDE
  FG_ICCOMP_LDFLAGS="-L$FG_ICCOMP_DIR_LIB"
  FG_ICCOMP_LIBS="-liccomp2x -lfl"
  # save values
  TMP_CXXFLAGS=$CXXFLAGS
  TMP_CPPFLAGS=$CPPFLAGS
  TMP_LDFLAGS=$LDFLAGS
  TMP_LIBS=$LIBS
  # set linker options for a local test 
  CXXFLAGS=$FG_ICCOMP_CXXFLAGS
  LDFLAGS=$FG_ICCOMP_LDFLAGS
  LIBS=$FG_ICCOMP_LIBS
  # test a single function of the SBS Vic2500 bus driver
  AC_LANG_PUSH(C++)		
  AC_LINK_IFELSE(
    [AC_LANG_PROGRAM([#include "ic-comp2x.h"],
                     [ICcomp2x fg;])],
    have_lib_iccomp2x=yes,
    have_lib_iccomp2x=no)
  AC_LANG_POP(C++)		
  # put the saved values
  CXXFLAGS=$TMP_CXXFLAGS
  CPPFLAGS=$TMP_CPPFLAGS
  LDFLAGS=$TMP_LDFLAGS
  LIBS=$TMP_LIBS

  if test "x$have_lib_iccomp2x" = "xyes"; then
    # Vic2500 robot works
    CXXFLAGS="$CXXFLAGS $FG_ICCOMP_CXXFLAGS"
    LDFLAGS="$LDFLAGS $FG_ICCOMP_LDFLAGS"
    LIBS="$FG_ICCOMP_LIBS $LIBS"
    AC_DEFINE([VISP_HAVE_ICCOMP], 1,
              [Define to 1 if the ICcomp framegrabber api is available])
    AC_MSG_RESULT(yes)
    AC_SUBST(FG_ICCOMP_CXXFLAGS)
    AC_SUBST(FG_ICCOMP_LDFLAGS)
    AC_SUBST(FG_ICCOMP_LIBS)
  else
    AC_MSG_RESULT(no)
  fi
else
  AC_MSG_RESULT(no)
fi

])