#############################################################################
#
# ViSP, open source Visual Servoing Platform software.
# Copyright (C) 2005 - 2019 by Inria. All rights reserved.
#
# This software is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
# See the file LICENSE.txt at the root directory of this source
# distribution for additional information about the GNU GPL.
#
# For using ViSP with software that can not be combined with the GNU
# GPL, please contact Inria about acquiring a ViSP Professional
# Edition License.
#
# See http://visp.inria.fr for more information.
#
# This software was developed at:
# Inria Rennes - Bretagne Atlantique
# Campus Universitaire de Beaulieu
# 35042 Rennes Cedex
# France
#
# If you have questions regarding the use of this file, please contact
# Inria at visp@inria.fr
#
# This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
# WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
#
# Description:
#   M4 macro for ViSP detection with configure tool.
#
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
# Authors:
#   Fabien Spindler, <Fabien.Spindler@irisa.fr>
#
#############################################################################



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
ac_visp_extrapath="/usr/bin"

AC_ARG_WITH([visp-install-bin],
AC_HELP_STRING([--with-visp-install-bin], [enable use of ViSP [[default=yes]]])
AC_HELP_STRING([--with-visp-install-bin=DIR], [give location of visp-config shell script (/usr/bin by default)]),
  [ case $withval in
    no)  ac_visp_desired=false ;;
    yes) ac_visp_desired=true ;;
    *)   ac_visp_desired=true
         ac_visp_extrapath=$withval ;;
    esac],
  [])

if $ac_visp_desired; then
  ac_visp_config_script="$ac_visp_extrapath/visp-config"

  AC_CHECK_FILE($ac_visp_config_script,[cv_visp_avail=true],
                 [cv_visp_avail=false])
  ac_visp_avail=$cv_visp_avail
fi

if $ac_visp_avail; then
  ac_visp_cflags="`$ac_visp_config_script --cflags`"
  ac_visp_libs="`$ac_visp_config_script --libs`"
  ifelse([$1], , :, [$1])
else
  ifelse([$2], , :, [$2])
fi
]) # AC_HAVE_VISP_IFELSE()

