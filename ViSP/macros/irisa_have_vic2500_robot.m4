dnl Calls IRISA_HAVE_VIC2500_ROBOT (contained in this file) as a subroutine.
AC_DEFUN([IRISA_HAVE_VIC2500_ROBOT],
[

#-----------------------------------------------------------------------
# Manage the --with-robot-Vic2500-dir option
AC_ARG_WITH([robot-Vic2500-dir],
  AC_HELP_STRING([--with-robot-Vic2500-dir=DIR],
		 [Location of Vic2500 robot api. By default /udd/fspindle/robot/Vic2500/current]),
                 path_robot_vic2500=$withval, 
                 path_robot_vic2500=/udd/fspindle/robot/Vic2500/current)
# check if the vic2500 robot home dir exists
#AC_CHECK_FILE([$path_robot_vic2500], [with_robot_vic2500=yes], [with_robot_vic2500=no])
with_robot_vic2500=yes
AC_MSG_CHECKING(whether Vic2500 robot api works)
# set some variables if the vic2500 robot is reacheable
if test "x$with_robot_vic2500" = "xyes"; then
  ROBOT_VIC2500_HOME=$path_robot_vic2500
  ROBOT_VIC2500_DIR_INCLUDE=$ROBOT_VIC2500_HOME/include
  ROBOT_VIC2500_DIR_LIB=$ROBOT_VIC2500_HOME/lib
  ROBOT_VIC2500_CXXFLAGS=-I$ROBOT_VIC2500_DIR_INCLUDE
  ROBOT_VIC2500_LDFLAGS="-L$ROBOT_VIC2500_DIR_LIB"
  ROBOT_VIC2500_LIBS="-lvic2500 -lvic2500serial"
  # save values
  TMP_CXXFLAGS=$CXXFLAGS
  TMP_CPPFLAGS=$CPPFLAGS
  TMP_LDFLAGS=$LDFLAGS
  TMP_LIBS=$LIBS
  # set linker options for a local test 
  CXXFLAGS=$ROBOT_VIC2500_CXXFLAGS
  LDFLAGS=$ROBOT_VIC2500_LDFLAGS
  LIBS=$ROBOT_VIC2500_LIBS
  # test a single function of the SBS Vic2500 bus driver
  AC_LANG_PUSH(C++)		
  AC_LINK_IFELSE(
    [AC_LANG_PROGRAM([char *name="foo"; char openserial(char *);],
                     [openserial(name)])],
    have_lib_serial=yes,
    have_lib_serial=no)
  AC_LANG_POP(C++)		
  # put the saved values
  CXXFLAGS=$TMP_CXXFLAGS
  CPPFLAGS=$TMP_CPPFLAGS
  LDFLAGS=$TMP_LDFLAGS
  LIBS=$TMP_LIBS

  if test "x$have_lib_serial" = "xyes"; then
    # Vic2500 robot works
    CXXFLAGS="$CXXFLAGS $ROBOT_VIC2500_CXXFLAGS"
    LDFLAGS="$LDFLAGS $ROBOT_VIC2500_LDFLAGS"
    LIBS="$LIBS $ROBOT_VIC2500_LIBS"
    AC_DEFINE([HAVE_ROBOT_VIC2500], 1,
              [Define to 1 if the Vic2500 robot api is available])
    AC_MSG_RESULT(yes)
    AC_SUBST(ROBOT_VIC2500_CXXFLAGS)
    AC_SUBST(ROBOT_VIC2500_LDFLAGS)
    AC_SUBST(ROBOT_VIC2500_LIBS)
  else
    AC_MSG_RESULT(no)
  fi
else
  AC_MSG_RESULT(no)
fi

])