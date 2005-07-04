############################################################################
# Usage:
#   AC_HAVE_VISP_IFELSE( IF-FOUND, IF-NOT-FOUND )
#
# Description:
#   This macro locates the ViSP development system.  If it is found,
#   the set of variables listed below are set up as described and made
#   available to the configure script.
#
# Autoconf Variables:
# > $ac_visp_desired     true | false (defaults to true)
# < $ac_visp_avail       true | false
# < $ac_visp_cxxflags    (extra flags the C++ compiler needs)
# < $ac_visp_ldflags     (extra flags the linker needs)
# < $ac_visp_libs        (link library flags the linker needs)
#
# Author:
#   Fabien Spindler, <Fabien.Spindler@irisa.fr>
#
# TODO:
#



AC_DEFUN([AC_HAVE_VISP_IFELSE], [
AC_PREREQ([2.14a])

# official variables
ac_visp_avail=false
ac_visp_cppflags=
ac_visp_cflags=
ac_visp_cxxflags=
ac_visp_ldflags=
ac_visp_libs=

# internal variables
ac_visp_desired=true
ac_visp_extrapath="/local/soft/ViSP/ViSP-2"

AC_ARG_WITH([visp],
AC_HELP_STRING([--with-visp], [enable use of ViSP [[default=yes]]])
AC_HELP_STRING([--with-visp=DIR], [give prefix location of ViSP]),
  [ case $withval in
    no)  ac_visp_desired=false ;;
    yes) ac_visp_desired=true ;;
    *)   ac_visp_desired=true
         ac_visp_extrapath=$withval ;;
    esac],
  [])

if $ac_visp_desired; then

    AC_CACHE_CHECK(
      [if we can compile and link with the ViSP library],
      cv_visp_avail,
      [ac_save_cxxflags=$CXXFLAGS
      ac_save_ldflags=$LDFLAGS
      ac_save_libs=$LIBS

      ac_visp_cxxflags="-I$ac_visp_extrapath/include"
      ac_visp_ldflags="-L$ac_visp_extrapath/lib"
      ac_visp_libs="-lvisp"

      CXXFLAGS="$CXXFLAGS $ac_visp_cxxflags"
      LDFLAGS="$LDFLAGS $ac_visp_ldflags"
      LIBS="$ac_visp_libs $LIBS"

      AC_LANG_PUSH(C++)

      AC_TRY_LINK(
        [#include <visp/vpMath.h>],
        [vpMath::fact(2);],
        [cv_visp_avail=true],
        [cv_visp_avail=false])

      AC_LANG_POP

      CXXFLAGS=$ac_save_cxxflags
      LDFLAGS=$ac_save_ldflags
      LIBS=$ac_save_libs
    ])
    ac_visp_avail=$cv_visp_avail
fi

if $ac_visp_avail; then
  ifelse([$1], , :, [$1])
else
  ifelse([$2], , :, [$2])
fi
]) # AC_HAVE_VISP_IFELSE()

