#############################################################################
#
# $Id: have_visp2.m4,v 1.4 2006-05-30 08:35:43 fspindle Exp $
#
# Copyright (C) 1998-2006 Inria. All rights reserved.
#
# This software was developed at:
# IRISA/INRIA Rennes
# Projet Lagadic
# Campus Universitaire de Beaulieu
# 35042 Rennes Cedex
# http://www.irisa.fr/lagadic
#
# This file is part of the ViSP toolkit
#
# This file may be distributed under the terms of the Q Public License
# as defined by Trolltech AS of Norway and appearing in the file
# LICENSE included in the packaging of this file.
#
# Licensees holding valid ViSP Professional Edition licenses may
# use this file in accordance with the ViSP Commercial License
# Agreement provided with the Software.
#
# This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
# WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
#
# Contact visp@irisa.fr if any conditions of this licensing are
# not clear to you.
#
# Description:
#   M4 macro for ViSP-2 detection with configure tool.
#
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
# Authors:
#   Fabien Spindler, <Fabien.Spindler@irisa.fr>
#
#############################################################################



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
ac_visp2_extrapath="/usr/bin"

AC_ARG_WITH([visp-install-bin],
AC_HELP_STRING([--with-visp-install-bin], [enable use of ViSP2 [[default=yes]]])
AC_HELP_STRING([--with-visp-install-bin=DIR], [give location of visp-config shell script (/usr/bin by default)]),
  [ case $withval in
    no)  ac_visp2_desired=false ;;
    yes) ac_visp2_desired=true ;;
    *)   ac_visp2_desired=true
         ac_visp2_extrapath=$withval ;;
    esac],
  [])

if $ac_visp2_desired; then
  ac_visp2_config_script="$ac_visp2_extrapath/visp-config"

  AC_CHECK_FILE($ac_visp2_config_script,[cv_visp2_avail=true],
                 [cv_visp2_avail=false])
  ac_visp2_avail=$cv_visp2_avail
fi

if $ac_visp2_avail; then
  ac_visp2_cflags="`$ac_visp2_config_script --cflags`"
  ac_visp2_libs="`$ac_visp2_config_script --libs`"
  ifelse([$1], , :, [$1])
else
  ifelse([$2], , :, [$2])
fi
]) # AC_HAVE_VISP2_IFELSE()

