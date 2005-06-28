dnl Calls IRISA_HAVE_ROBOT_PTUEVI (contained in this file) as a subroutine.
AC_DEFUN([IRISA_HAVE_ROBOT_PTUEVI],
[

#-----------------------------------------------------------------------
# Manage the --with-robot-PtuEvi-dir option
AC_ARG_WITH([robot-PtuEvi-dir],
  AC_HELP_STRING([--with-robot-PtuEvi=DIR],
		 [Location of PtuEvi robot api. By default /udd/fspindle/robot/Ptu-Evi/current]),
                 path_robot_ptuevi=$withval, 
                 path_robot_ptuevi=/udd/fspindle/robot/Ptu-Evi/current)
# check if the ptuevi robot home dir exists
#AC_CHECK_FILE([$path_robot_ptuevi], [with_robot_ptuevi=yes], [with_robot_ptuevi=no])
with_robot_ptuevi=yes
AC_MSG_CHECKING(whether PtuEvi robot api works)
# set some variables if the ptuevi robot is reacheable
if test "x$with_robot_ptuevi" = "xyes"; then
  ROBOT_PTUEVI_HOME=$path_robot_ptuevi
  ROBOT_PTUEVI_DIR_INCLUDE=$ROBOT_PTUEVI_HOME/include
  ROBOT_PTUEVI_DIR_LIB=$ROBOT_PTUEVI_HOME/lib
  ROBOT_PTUEVI_CXXFLAGS=-I$ROBOT_PTUEVI_DIR_INCLUDE
  ROBOT_PTUEVI_LDFLAGS="-L$ROBOT_PTUEVI_DIR_LIB"
  ROBOT_PTUEVI_LIBS="-lptu -levi -lserial"
  # save values
  TMP_CXXFLAGS=$CXXFLAGS
  TMP_CPPFLAGS=$CPPFLAGS
  TMP_LDFLAGS=$LDFLAGS
  TMP_LIBS=$LIBS
  # set linker options for a local test 
  CXXFLAGS=$ROBOT_PTUEVI_CXXFLAGS
  LDFLAGS=$ROBOT_PTUEVI_LDFLAGS
  LIBS=$ROBOT_PTUEVI_LIBS
  # test a single function of the SBS Ptuevi bus driver
  AC_LANG_PUSH(C++)		
  AC_LINK_IFELSE(
    [AC_LANG_PROGRAM([char *name="foo"; int speed = 19200; char openserial(char *, int );],
                     [openserial(name, speed)])],
    have_lib_serial=yes,
    have_lib_serial=no)
  AC_LANG_POP(C++)		
  # put the saved values
  CXXFLAGS=$TMP_CXXFLAGS
  CPPFLAGS=$TMP_CPPFLAGS
  LDFLAGS=$TMP_LDFLAGS
  LIBS=$TMP_LIBS

  if test "x$have_lib_serial" = "xyes"; then
    # Ptuevi robot works
    CXXFLAGS="$CXXFLAGS $ROBOT_PTUEVI_CXXFLAGS"
    LDFLAGS="$LDFLAGS $ROBOT_PTUEVI_LDFLAGS"
    LIBS="$ROBOT_PTUEVI_LIBS $LIBS"
    AC_DEFINE([HAVE_ROBOT_PTUEVI], 1,
              [Define to 1 if the Ptu-Evi robot api is available])
    AC_MSG_RESULT(yes)
    AC_SUBST(ROBOT_PTUEVI_CXXFLAGS)
    AC_SUBST(ROBOT_PTUEVI_LDFLAGS)
    AC_SUBST(ROBOT_PTUEVI_LIBS)
  else
    AC_MSG_RESULT(no)
  fi
else
  AC_MSG_RESULT(no)
fi

])