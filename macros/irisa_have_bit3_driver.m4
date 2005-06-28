dnl Calls IRISA_HAVE_BIT3_DRIVER (contained in this file) as a subroutine.
AC_DEFUN([IRISA_HAVE_BIT3_DRIVER],
[

#-----------------------------------------------------------------------
# Manage Bit3 driver with --with-bit3-dir option
AC_ARG_WITH([bit3-dir],
  AC_HELP_STRING([--with-bit3-dir=DIR],
		 [Location of SBS Bit3 driver. By default /udd/fspindle/robot/driver/bit3-617/1003/current]),
                 path_bit3=$withval, 
                 path_bit3=/udd/fspindle/robot/driver/bit3-617/1003/current)
# check if the bit3 home dir exists
#AC_CHECK_FILE([$path_bit3], [with_bit3=yes], [with_bit3=no])
with_bit3=yes
AC_MSG_CHECKING(whether SBS Bit3 bus driver works)
# set some variables if the bit3 driver is reacheable
if test "x$with_bit3" = "xyes"; then
  BIT3_HOME=$path_bit3
  BIT3_DIR_INCLUDE=$BIT3_HOME/include
  BIT3_DIR_LIB=$BIT3_HOME/include
  BIT3_CXXFLAGS=-I$BIT3_DIR_INCLUDE
  BIT3_CPPFLAGS="-DBT1003"
  BIT3_LDFLAGS="-L$BIT3_DIR_LIB"
  BIT3_LIBS="-lbtp -lpthread"
  # save values
  TMP_CXXFLAGS=$CXXFLAGS
  TMP_CPPFLAGS=$CPPFLAGS
  TMP_LDFLAGS=$LDFLAGS
  TMP_LIBS=$LIBS
  # set linker options for a local test 
  CXXFLAGS=$BIT3_CXXFLAGS
  CPPFLAGS=$BIT3_CPPFLAGS
  LDFLAGS=$BIT3_LDFLAGS
  LIBS=$BIT3_LIBS
  # test a single function of the SBS Bit3 bus driver
  #AC_LANG_PUSH(C++)		
  AC_LINK_IFELSE(
    [AC_LANG_PROGRAM([char *name="foo"; char bt_str2dev(char *);],
                     [bt_str2dev(name)])],
    have_lib_btp=yes,
    have_lib_btp=no)
  # put the saved values
  CXXFLAGS=$TMP_CXXFLAGS
  CPPFLAGS=$TMP_CPPFLAGS
  LDFLAGS=$TMP_LDFLAGS
  LIBS=$TMP_LIBS

  if test "x$have_lib_btp" = "xyes"; then
    # Bit3 robot works
    CXXFLAGS="$CXXFLAGS $BIT3_CXXFLAGS"
    CPPFLAGS="$CPPFLAGS $BIT3_CPPFLAGS"
    LDFLAGS="$LDFLAGS $BIT3_LDFLAGS"
    LIBS="$BIT3_LIBS $LIBS"
    AC_DEFINE([HAVE_BIT3], 1,
              [Define to 1 if the SBS Bit3 bus driver is available])
    AC_MSG_RESULT(yes)
    AC_SUBST(BIT3_CXXFLAGS)
    AC_SUBST(BIT3_CPPFLAGS)
    AC_SUBST(BIT3_LDFLAGS)
    AC_SUBST(BIT3_LIBS)
  else
    AC_MSG_RESULT(no)
  fi
else
  AC_MSG_RESULT(no)
fi

])