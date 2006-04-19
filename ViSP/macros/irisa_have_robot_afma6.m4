dnl Calls IRISA_HAVE_ROBOT_AFMA6 (contained in this file) as a subroutine.
AC_DEFUN([IRISA_HAVE_ROBOT_AFMA6],
[
#-----------------------------------------------------------------------
# Manage Afma6 robot with --with-robot-Afma6 option
AC_ARG_WITH([robot-Afma6],
  AC_HELP_STRING([--with-robot-Afma6=DIR],
		 [Location of Afma6 robot api. By default /udd/fspindle/robot/Afma6/current]),
                 path_robot_afma6=$withval, 
                 path_robot_afma6=/udd/fspindle/robot/Afma6/current)
# check if the afma6 robot home dir exists
#AC_CHECK_FILE([$path_robot_afma6], [with_robot_afma6=yes], [with_robot_afma6=no])
with_robot_afma6=yes
AC_MSG_CHECKING(whether Afma6 robot api works)
# set some variables if the afma6 robot is reacheable
if test "x$with_robot_afma6" = "xyes"; then
  ROBOT_AFMA6_HOME=$path_robot_afma6
  ROBOT_AFMA6_DIR_INCLUDE=$ROBOT_AFMA6_HOME/include
  ROBOT_AFMA6_DIR_LIB=$ROBOT_AFMA6_HOME/lib
  ROBOT_AFMA6_CXXFLAGS=-I$ROBOT_AFMA6_DIR_INCLUDE
  ROBOT_AFMA6_LDFLAGS="-L$ROBOT_AFMA6_DIR_LIB"
  ROBOT_AFMA6_LIBS="-lrobotAfma6 -ltoolsAfma6 -luprimAfma6 -lbit3Afma6"
  # save values
  TMP_CXXFLAGS=$CXXFLAGS
  TMP_CPPFLAGS=$CPPFLAGS
  TMP_LDFLAGS=$LDFLAGS
  TMP_LIBS=$LIBS
  # set linker options for a local test 
  CXXFLAGS=$ROBOT_AFMA6_CXXFLAGS
  LDFLAGS=$ROBOT_AFMA6_LDFLAGS
  LIBS=$ROBOT_AFMA6_LIBS
  # test a single function of the Afma4 api
  AC_LANG_PUSH(C++)		
  AC_LINK_IFELSE(
    [AC_LANG_PROGRAM(,)],
    have_lib_toolsAfma6=yes,
    have_lib_toolsAfma6=no)
  AC_LANG_POP(C++)		
  # put the saved values
  CXXFLAGS=$TMP_CXXFLAGS
  CPPFLAGS=$TMP_CPPFLAGS
  LDFLAGS=$TMP_LDFLAGS
  LIBS=$TMP_LIBS

  if test "x$have_lib_toolsAfma6" = "xyes"; then
    # Afma6 robot works
    CXXFLAGS="$CXXFLAGS $ROBOT_AFMA6_CXXFLAGS"
    LDFLAGS="$LDFLAGS $ROBOT_AFMA6_LDFLAGS"
    LIBS="$ROBOT_AFMA6_LIBS $LIBS"
    AC_DEFINE([VISP_HAVE_AFMA6], 1,
              [Define to 1 if the Afma6 robot api are available])
    AC_MSG_RESULT(yes)
    AC_SUBST(ROBOT_AFMA6_CXXFLAGS)
    AC_SUBST(ROBOT_AFMA6_LDFLAGS)
    AC_SUBST(ROBOT_AFMA6_LIBS)
  else
    AC_MSG_RESULT(no)
  fi
else
  AC_MSG_RESULT(no)
fi


])