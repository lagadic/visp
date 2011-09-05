# Process this file with autoconf to produce a configure script.
AC_PREREQ(2.50)
AC_INIT(HelloWorld, 1.0)
#AM_ACLOCAL_INCLUDE(macros)
AM_INIT_AUTOMAKE
AC_PROG_CXX
AC_CONFIG_FILES([Makefile])

# check for ViSP library
AC_HAVE_VISP_IFELSE(have_visp=yes,have_visp=no)
if test "x$have_visp" = "xyes"; then
  CXXFLAGS="$CXXFLAGS $ac_visp_cflags "
  LIBS="$LIBS $ac_visp_libs "
fi

AC_OUTPUT