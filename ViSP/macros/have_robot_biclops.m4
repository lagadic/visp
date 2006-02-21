dnl Calls HAVE_ROBOT_BICLOPS (contained in this file) as a subroutine.
AC_DEFUN([HAVE_ROBOT_BICLOPS],
[
#-----------------------------------------------------------------------
# check if libBiclops, libPMD, libUtils are reachable
AC_MSG_CHECKING(whether biclops pan-tilt api works)
  
  ROBOT_BICLOPS_LIBS="-lBiclops -lPMD -lUtils"
  # save values
  TMP_LIBS=$LIBS
  # set linker options for a local test 
  LIBS=$ROBOT_PTUEVI_LIBS
  # test a single function of the SBS Ptuevi bus driver
  AC_LANG_PUSH(C++)		
  AC_LINK_IFELSE(
    [AC_LANG_PROGRAM(,)],
    have_robot_biclops=yes,
    have_robot_biclops=no)
  AC_LANG_POP(C++)		
  # put the saved values
  LIBS=$TMP_LIBS

  if test "x$have_robot_biclops" = "xyes"; then
    # Biclops robot works
    LIBS="$ROBOT_BICLOPS_LIBS $LIBS"
    AC_DEFINE([HAVE_ROBOT_BICLOPS_PT], 1,
              [Define to 1 if the biclops pan-tilt api is available])
    AC_MSG_RESULT(yes)
  else
    AC_MSG_RESULT(no)
  fi

])
