dnl Calls IRISA_HAVE_AFMA4_ROBOT (contained in this file) as a subroutine.
AC_DEFUN([IRISA_HAVE_AFMA4_ROBOT],
[

#-----------------------------------------------------------------------
# Manage Afma4 robot with --with-robot-Afma4-dir option
AC_ARG_WITH([robot-Afma4-dir],
  AC_HELP_STRING([--with-robot-Afma4=DIR],
		 [Location of Afma4 robot api. By default /udd/fspindle/robot/Afma4/current]),
                 path_robot_afma4=$withval, 
                 path_robot_afma4=/udd/fspindle/robot/Afma4/current)
# check if the afma4 robot home dir exists
#AC_CHECK_FILE([$path_robot_afma4], [with_robot_afma4=yes], [with_robot_afma4=no])
with_robot_afma4=yes
AC_MSG_CHECKING(whether Afma4 robot api works)
# set some variables if the afma4 robot is reacheable
if test "x$with_robot_afma4" = "xyes"; then
  ROBOT_AFMA4_HOME=$path_robot_afma4
  ROBOT_AFMA4_DIR_INCLUDE=$ROBOT_AFMA4_HOME/include
  ROBOT_AFMA4_DIR_LIB=$ROBOT_AFMA4_HOME/lib
  ROBOT_AFMA4_CXXFLAGS=-I$ROBOT_AFMA4_DIR_INCLUDE
  ROBOT_AFMA4_LDFLAGS="-L$ROBOT_AFMA4_DIR_LIB"
  ROBOT_AFMA4_LIBS="-lrobotAfma4 -ltoolsAfma4 -luprimAfma4 -lbit3Afma4 -lservolensAfma4"
  # save values
  TMP_CXXFLAGS=$CXXFLAGS
  TMP_CPPFLAGS=$CPPFLAGS
  TMP_LDFLAGS=$LDFLAGS
  TMP_LIBS=$LIBS
  # set linker options for a local test 
  CXXFLAGS=$ROBOT_AFMA4_CXXFLAGS
  LDFLAGS=$ROBOT_AFMA4_LDFLAGS
  LIBS=$ROBOT_AFMA4_LIBS
  # test a single function of the Afma4 api
  AC_LANG_PUSH(C++)		
  AC_LINK_IFELSE(
    [AC_LANG_PROGRAM([void mesure_temps();],[mesure_temps()])],
    have_lib_toolsAfma4=yes,
    have_lib_toolsAfma4=no)
  AC_LANG_POP(C++)		
  # put the saved values
  CXXFLAGS=$TMP_CXXFLAGS
  CPPFLAGS=$TMP_CPPFLAGS
  LDFLAGS=$TMP_LDFLAGS
  LIBS=$TMP_LIBS

  if test "x$have_lib_toolsAfma4" = "xyes"; then
    # Afma4 robot works
    CXXFLAGS="$CXXFLAGS $ROBOT_AFMA4_CXXFLAGS"
    LDFLAGS="$LDFLAGS $ROBOT_AFMA4_LDFLAGS"
    LIBS="$ROBOT_AFMA4_LIBS $LIBS"
    AC_DEFINE([HAVE_ROBOT_AFMA4], 1,
              [Define to 1 if the Afma4 robot api are available])
    AC_MSG_RESULT(yes)
    AC_SUBST(ROBOT_AFMA4_CXXFLAGS)
    AC_SUBST(ROBOT_AFMA4_LDFLAGS)
    AC_SUBST(ROBOT_AFMA4_LIBS)
  else
    AC_MSG_RESULT(no)
  fi
else
  AC_MSG_RESULT(no)
fi
])