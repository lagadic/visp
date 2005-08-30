############################################################################
# Usage:
#   AC_HAVE_VISP2_IFELSE( IF-FOUND, IF-NOT-FOUND )
#
# Description:
#   This macro locates the ViSP2 development system.  If it is found,
#   the set of variables listed below are set up as described and made
#   available to the configure script.
#
# Autoconf Variables:
# > $ac_visp2_desired     true | false (defaults to true)
# < $ac_visp2_avail       true | false
# < $ac_visp2_cxxflags    (extra flags the C++ compiler needs)
# < $ac_visp2_ldflags     (extra flags the linker needs)
# < $ac_visp2_libs        (link library flags the linker needs)
#
# Author:
#   Fabien Spindler, <Fabien.Spindler@irisa.fr>
#
# TODO:
#



AC_DEFUN([AC_HAVE_VISP2_IFELSE], [
AC_PREREQ([2.14a])

# official variables
ac_visp2_avail=false
ac_visp2_cppflags=
ac_visp2_cflags=
ac_visp2_cxxflags=
ac_visp2_ldflags=
ac_visp2_libs=

# internal variables
ac_visp2_desired=true
ac_visp2_extrapath="/local/soft/ViSP/ViSP-2"

AC_ARG_WITH([visp2],
AC_HELP_STRING([--with-visp2], [enable use of ViSP2 [[default=yes]]])
AC_HELP_STRING([--with-visp2=DIR], [give prefix location of ViSP2]),
  [ case $withval in
    no)  ac_visp2_desired=false ;;
    yes) ac_visp2_desired=true ;;
    *)   ac_visp2_desired=true
         ac_visp2_extrapath=$withval ;;
    esac],
  [])

if $ac_visp2_desired; then

    AC_CACHE_CHECK(
      [if we can compile and link with the ViSP2 library],
      cv_visp2_avail,
      [ac_save_cxxflags=$CXXFLAGS
      ac_save_ldflags=$LDFLAGS
      ac_save_libs=$LIBS

      ac_visp2_cxxflags="-I$ac_visp2_extrapath/include"
      ac_visp2_ldflags="-L$ac_visp2_extrapath/lib"
      ac_visp2_libs="-lvisp-2"

      CXXFLAGS="$CXXFLAGS $ac_visp2_cxxflags"
      LDFLAGS="$LDFLAGS $ac_visp2_ldflags"
      LIBS="$ac_visp2_libs $LIBS"

      AC_LANG_PUSH(C++)

      AC_TRY_LINK(
        [#include <visp/vpMath.h>],
        [vpMath::fact(2);],
        [cv_visp2_avail=true],
        [cv_visp2_avail=false])

      AC_LANG_POP

      CXXFLAGS=$ac_save_cxxflags
      LDFLAGS=$ac_save_ldflags
      LIBS=$ac_save_libs
    ])
    ac_visp2_avail=$cv_visp2_avail
fi

if $ac_visp2_avail; then
  ifelse([$1], , :, [$1])
else
  ifelse([$2], , :, [$2])
fi
]) # AC_HAVE_VISP2_IFELSE()

