# aclocal.m4 generated automatically by aclocal 1.6.3 -*- Autoconf -*-

# Copyright 1996, 1997, 1998, 1999, 2000, 2001, 2002
# Free Software Foundation, Inc.
# This file is free software; the Free Software Foundation
# gives unlimited permission to copy and/or distribute it,
# with or without modifications, as long as this notice is preserved.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY, to the extent permitted by law; without
# even the implied warranty of MERCHANTABILITY or FITNESS FOR A
# PARTICULAR PURPOSE.

# aclocal-include.m4
# 
# This macro adds the name macrodir to the set of directories
# that `aclocal' searches for macros.  

# serial 1

dnl AM_ACLOCAL_INCLUDE(macrodir)
AC_DEFUN([AM_ACLOCAL_INCLUDE],
[
	AM_CONDITIONAL(INSIDE_GNOME_COMMON, test x = y)

	test -n "$ACLOCAL_FLAGS" && ACLOCAL="$ACLOCAL $ACLOCAL_FLAGS"

	for k in $1 ; do ACLOCAL="$ACLOCAL -I $k" ; done
])

# AM_CONDITIONAL                                              -*- Autoconf -*-

# Copyright 1997, 2000, 2001 Free Software Foundation, Inc.

# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
# any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
# 02111-1307, USA.

# serial 5

AC_PREREQ(2.52)

# AM_CONDITIONAL(NAME, SHELL-CONDITION)
# -------------------------------------
# Define a conditional.
AC_DEFUN([AM_CONDITIONAL],
[ifelse([$1], [TRUE],  [AC_FATAL([$0: invalid condition: $1])],
        [$1], [FALSE], [AC_FATAL([$0: invalid condition: $1])])dnl
AC_SUBST([$1_TRUE])
AC_SUBST([$1_FALSE])
if $2; then
  $1_TRUE=
  $1_FALSE='#'
else
  $1_TRUE='#'
  $1_FALSE=
fi
AC_CONFIG_COMMANDS_PRE(
[if test -z "${$1_TRUE}" && test -z "${$1_FALSE}"; then
  AC_MSG_ERROR([conditional \"$1\" was never defined.
Usually this means the macro was only invoked conditionally.])
fi])])

# Configure paths for GTK+
# Owen Taylor     1997-2001

dnl AM_PATH_GTK_2_0([MINIMUM-VERSION, [ACTION-IF-FOUND [, ACTION-IF-NOT-FOUND [, MODULES]]]])
dnl Test for GTK+, and define GTK_CFLAGS and GTK_LIBS, if gthread is specified in MODULES, 
dnl pass to pkg-config
dnl
AC_DEFUN([AM_PATH_GTK_2_0],
[dnl 
dnl Get the cflags and libraries from pkg-config
dnl
AC_ARG_ENABLE(gtktest, [  --disable-gtktest       do not try to compile and run a test GTK+ program],
		    , enable_gtktest=yes)

  pkg_config_args=gtk+-2.0
  for module in . $4
  do
      case "$module" in
         gthread) 
             pkg_config_args="$pkg_config_args gthread-2.0"
         ;;
      esac
  done

  no_gtk=""

  AC_PATH_PROG(PKG_CONFIG, pkg-config, no)

  if test x$PKG_CONFIG != xno ; then
    if pkg-config --atleast-pkgconfig-version 0.7 ; then
      :
    else
      echo "*** pkg-config too old; version 0.7 or better required."
      no_gtk=yes
      PKG_CONFIG=no
    fi
  else
    no_gtk=yes
  fi

  min_gtk_version=ifelse([$1], ,2.0.0,$1)
  AC_MSG_CHECKING(for GTK+ - version >= $min_gtk_version)

  if test x$PKG_CONFIG != xno ; then
    ## don't try to run the test against uninstalled libtool libs
    if $PKG_CONFIG --uninstalled $pkg_config_args; then
	  echo "Will use uninstalled version of GTK+ found in PKG_CONFIG_PATH"
	  enable_gtktest=no
    fi

    if $PKG_CONFIG --atleast-version $min_gtk_version $pkg_config_args; then
	  :
    else
	  no_gtk=yes
    fi
  fi

  if test x"$no_gtk" = x ; then
    GTK_CFLAGS=`$PKG_CONFIG $pkg_config_args --cflags`
    GTK_LIBS=`$PKG_CONFIG $pkg_config_args --libs`
    gtk_config_major_version=`$PKG_CONFIG --modversion gtk+-2.0 | \
           sed 's/\([[0-9]]*\).\([[0-9]]*\).\([[0-9]]*\)/\1/'`
    gtk_config_minor_version=`$PKG_CONFIG --modversion gtk+-2.0 | \
           sed 's/\([[0-9]]*\).\([[0-9]]*\).\([[0-9]]*\)/\2/'`
    gtk_config_micro_version=`$PKG_CONFIG --modversion gtk+-2.0 | \
           sed 's/\([[0-9]]*\).\([[0-9]]*\).\([[0-9]]*\)/\3/'`
    if test "x$enable_gtktest" = "xyes" ; then
      ac_save_CFLAGS="$CFLAGS"
      ac_save_LIBS="$LIBS"
      CFLAGS="$CFLAGS $GTK_CFLAGS"
      LIBS="$GTK_LIBS $LIBS"
dnl
dnl Now check if the installed GTK+ is sufficiently new. (Also sanity
dnl checks the results of pkg-config to some extent)
dnl
      rm -f conf.gtktest
      AC_TRY_RUN([
#include <gtk/gtk.h>
#include <stdio.h>
#include <stdlib.h>

int 
main ()
{
  int major, minor, micro;
  char *tmp_version;

  system ("touch conf.gtktest");

  /* HP/UX 9 (%@#!) writes to sscanf strings */
  tmp_version = g_strdup("$min_gtk_version");
  if (sscanf(tmp_version, "%d.%d.%d", &major, &minor, &micro) != 3) {
     printf("%s, bad version string\n", "$min_gtk_version");
     exit(1);
   }

  if ((gtk_major_version != $gtk_config_major_version) ||
      (gtk_minor_version != $gtk_config_minor_version) ||
      (gtk_micro_version != $gtk_config_micro_version))
    {
      printf("\n*** 'pkg-config --modversion gtk+-2.0' returned %d.%d.%d, but GTK+ (%d.%d.%d)\n", 
             $gtk_config_major_version, $gtk_config_minor_version, $gtk_config_micro_version,
             gtk_major_version, gtk_minor_version, gtk_micro_version);
      printf ("*** was found! If pkg-config was correct, then it is best\n");
      printf ("*** to remove the old version of GTK+. You may also be able to fix the error\n");
      printf("*** by modifying your LD_LIBRARY_PATH enviroment variable, or by editing\n");
      printf("*** /etc/ld.so.conf. Make sure you have run ldconfig if that is\n");
      printf("*** required on your system.\n");
      printf("*** If pkg-config was wrong, set the environment variable PKG_CONFIG_PATH\n");
      printf("*** to point to the correct configuration files\n");
    } 
  else if ((gtk_major_version != GTK_MAJOR_VERSION) ||
	   (gtk_minor_version != GTK_MINOR_VERSION) ||
           (gtk_micro_version != GTK_MICRO_VERSION))
    {
      printf("*** GTK+ header files (version %d.%d.%d) do not match\n",
	     GTK_MAJOR_VERSION, GTK_MINOR_VERSION, GTK_MICRO_VERSION);
      printf("*** library (version %d.%d.%d)\n",
	     gtk_major_version, gtk_minor_version, gtk_micro_version);
    }
  else
    {
      if ((gtk_major_version > major) ||
        ((gtk_major_version == major) && (gtk_minor_version > minor)) ||
        ((gtk_major_version == major) && (gtk_minor_version == minor) && (gtk_micro_version >= micro)))
      {
        return 0;
       }
     else
      {
        printf("\n*** An old version of GTK+ (%d.%d.%d) was found.\n",
               gtk_major_version, gtk_minor_version, gtk_micro_version);
        printf("*** You need a version of GTK+ newer than %d.%d.%d. The latest version of\n",
	       major, minor, micro);
        printf("*** GTK+ is always available from ftp://ftp.gtk.org.\n");
        printf("***\n");
        printf("*** If you have already installed a sufficiently new version, this error\n");
        printf("*** probably means that the wrong copy of the pkg-config shell script is\n");
        printf("*** being found. The easiest way to fix this is to remove the old version\n");
        printf("*** of GTK+, but you can also set the PKG_CONFIG environment to point to the\n");
        printf("*** correct copy of pkg-config. (In this case, you will have to\n");
        printf("*** modify your LD_LIBRARY_PATH enviroment variable, or edit /etc/ld.so.conf\n");
        printf("*** so that the correct libraries are found at run-time))\n");
      }
    }
  return 1;
}
],, no_gtk=yes,[echo $ac_n "cross compiling; assumed OK... $ac_c"])
       CFLAGS="$ac_save_CFLAGS"
       LIBS="$ac_save_LIBS"
     fi
  fi
  if test "x$no_gtk" = x ; then
     AC_MSG_RESULT(yes (version $gtk_config_major_version.$gtk_config_minor_version.$gtk_config_micro_version))
     ifelse([$2], , :, [$2])     
  else
     AC_MSG_RESULT(no)
     if test "$PKG_CONFIG" = "no" ; then
       echo "*** A new enough version of pkg-config was not found."
       echo "*** See http://pkgconfig.sourceforge.net"
     else
       if test -f conf.gtktest ; then
        :
       else
          echo "*** Could not run GTK+ test program, checking why..."
	  ac_save_CFLAGS="$CFLAGS"
	  ac_save_LIBS="$LIBS"
          CFLAGS="$CFLAGS $GTK_CFLAGS"
          LIBS="$LIBS $GTK_LIBS"
          AC_TRY_LINK([
#include <gtk/gtk.h>
#include <stdio.h>
],      [ return ((gtk_major_version) || (gtk_minor_version) || (gtk_micro_version)); ],
        [ echo "*** The test program compiled, but did not run. This usually means"
          echo "*** that the run-time linker is not finding GTK+ or finding the wrong"
          echo "*** version of GTK+. If it is not finding GTK+, you'll need to set your"
          echo "*** LD_LIBRARY_PATH environment variable, or edit /etc/ld.so.conf to point"
          echo "*** to the installed location  Also, make sure you have run ldconfig if that"
          echo "*** is required on your system"
	  echo "***"
          echo "*** If you have an old version installed, it is best to remove it, although"
          echo "*** you may also be able to get things to work by modifying LD_LIBRARY_PATH" ],
        [ echo "*** The test program failed to compile or link. See the file config.log for the"
          echo "*** exact error that occured. This usually means GTK+ is incorrectly installed."])
          CFLAGS="$ac_save_CFLAGS"
          LIBS="$ac_save_LIBS"
       fi
     fi
     GTK_CFLAGS=""
     GTK_LIBS=""
     ifelse([$3], , :, [$3])
  fi
  AC_SUBST(GTK_CFLAGS)
  AC_SUBST(GTK_LIBS)
  rm -f conf.gtktest
])

# Configure paths for GTK+
# Owen Taylor     97-11-3

dnl AM_PATH_GTK([MINIMUM-VERSION, [ACTION-IF-FOUND [, ACTION-IF-NOT-FOUND [, MODULES]]]])
dnl Test for GTK, and define GTK_CFLAGS and GTK_LIBS
dnl
AC_DEFUN(AM_PATH_GTK,
[dnl 
dnl Get the cflags and libraries from the gtk-config script
dnl
AC_ARG_WITH(gtk-prefix,[  --with-gtk-prefix=PFX   Prefix where GTK is installed (optional)],
            gtk_config_prefix="$withval", gtk_config_prefix="")
AC_ARG_WITH(gtk-exec-prefix,[  --with-gtk-exec-prefix=PFX Exec prefix where GTK is installed (optional)],
            gtk_config_exec_prefix="$withval", gtk_config_exec_prefix="")
AC_ARG_ENABLE(gtktest, [  --disable-gtktest       Do not try to compile and run a test GTK program],
		    , enable_gtktest=yes)

  for module in . $4
  do
      case "$module" in
         gthread) 
             gtk_config_args="$gtk_config_args gthread"
         ;;
      esac
  done

  if test x$gtk_config_exec_prefix != x ; then
     gtk_config_args="$gtk_config_args --exec-prefix=$gtk_config_exec_prefix"
     if test x${GTK_CONFIG+set} != xset ; then
        GTK_CONFIG=$gtk_config_exec_prefix/bin/gtk-config
     fi
  fi
  if test x$gtk_config_prefix != x ; then
     gtk_config_args="$gtk_config_args --prefix=$gtk_config_prefix"
     if test x${GTK_CONFIG+set} != xset ; then
        GTK_CONFIG=$gtk_config_prefix/bin/gtk-config
     fi
  fi

  AC_PATH_PROG(GTK_CONFIG, gtk-config, no)
  min_gtk_version=ifelse([$1], ,0.99.7,$1)
  AC_MSG_CHECKING(for GTK - version >= $min_gtk_version)
  no_gtk=""
  if test "$GTK_CONFIG" = "no" ; then
    no_gtk=yes
  else
    GTK_CFLAGS=`$GTK_CONFIG $gtk_config_args --cflags`
    GTK_LIBS=`$GTK_CONFIG $gtk_config_args --libs`
    gtk_config_major_version=`$GTK_CONFIG $gtk_config_args --version | \
           sed 's/\([[0-9]]*\).\([[0-9]]*\).\([[0-9]]*\)/\1/'`
    gtk_config_minor_version=`$GTK_CONFIG $gtk_config_args --version | \
           sed 's/\([[0-9]]*\).\([[0-9]]*\).\([[0-9]]*\)/\2/'`
    gtk_config_micro_version=`$GTK_CONFIG $gtk_config_args --version | \
           sed 's/\([[0-9]]*\).\([[0-9]]*\).\([[0-9]]*\)/\3/'`
    if test "x$enable_gtktest" = "xyes" ; then
      ac_save_CFLAGS="$CFLAGS"
      ac_save_LIBS="$LIBS"
      CFLAGS="$CFLAGS $GTK_CFLAGS"
      LIBS="$GTK_LIBS $LIBS"
dnl
dnl Now check if the installed GTK is sufficiently new. (Also sanity
dnl checks the results of gtk-config to some extent
dnl
      rm -f conf.gtktest
      AC_TRY_RUN([
#include <gtk/gtk.h>
#include <stdio.h>
#include <stdlib.h>

int 
main ()
{
  int major, minor, micro;
  char *tmp_version;

  system ("touch conf.gtktest");

  /* HP/UX 9 (%@#!) writes to sscanf strings */
  tmp_version = g_strdup("$min_gtk_version");
  if (sscanf(tmp_version, "%d.%d.%d", &major, &minor, &micro) != 3) {
     printf("%s, bad version string\n", "$min_gtk_version");
     exit(1);
   }

  if ((gtk_major_version != $gtk_config_major_version) ||
      (gtk_minor_version != $gtk_config_minor_version) ||
      (gtk_micro_version != $gtk_config_micro_version))
    {
      printf("\n*** 'gtk-config --version' returned %d.%d.%d, but GTK+ (%d.%d.%d)\n", 
             $gtk_config_major_version, $gtk_config_minor_version, $gtk_config_micro_version,
             gtk_major_version, gtk_minor_version, gtk_micro_version);
      printf ("*** was found! If gtk-config was correct, then it is best\n");
      printf ("*** to remove the old version of GTK+. You may also be able to fix the error\n");
      printf("*** by modifying your LD_LIBRARY_PATH enviroment variable, or by editing\n");
      printf("*** /etc/ld.so.conf. Make sure you have run ldconfig if that is\n");
      printf("*** required on your system.\n");
      printf("*** If gtk-config was wrong, set the environment variable GTK_CONFIG\n");
      printf("*** to point to the correct copy of gtk-config, and remove the file config.cache\n");
      printf("*** before re-running configure\n");
    } 
#if defined (GTK_MAJOR_VERSION) && defined (GTK_MINOR_VERSION) && defined (GTK_MICRO_VERSION)
  else if ((gtk_major_version != GTK_MAJOR_VERSION) ||
	   (gtk_minor_version != GTK_MINOR_VERSION) ||
           (gtk_micro_version != GTK_MICRO_VERSION))
    {
      printf("*** GTK+ header files (version %d.%d.%d) do not match\n",
	     GTK_MAJOR_VERSION, GTK_MINOR_VERSION, GTK_MICRO_VERSION);
      printf("*** library (version %d.%d.%d)\n",
	     gtk_major_version, gtk_minor_version, gtk_micro_version);
    }
#endif /* defined (GTK_MAJOR_VERSION) ... */
  else
    {
      if ((gtk_major_version > major) ||
        ((gtk_major_version == major) && (gtk_minor_version > minor)) ||
        ((gtk_major_version == major) && (gtk_minor_version == minor) && (gtk_micro_version >= micro)))
      {
        return 0;
       }
     else
      {
        printf("\n*** An old version of GTK+ (%d.%d.%d) was found.\n",
               gtk_major_version, gtk_minor_version, gtk_micro_version);
        printf("*** You need a version of GTK+ newer than %d.%d.%d. The latest version of\n",
	       major, minor, micro);
        printf("*** GTK+ is always available from ftp://ftp.gtk.org.\n");
        printf("***\n");
        printf("*** If you have already installed a sufficiently new version, this error\n");
        printf("*** probably means that the wrong copy of the gtk-config shell script is\n");
        printf("*** being found. The easiest way to fix this is to remove the old version\n");
        printf("*** of GTK+, but you can also set the GTK_CONFIG environment to point to the\n");
        printf("*** correct copy of gtk-config. (In this case, you will have to\n");
        printf("*** modify your LD_LIBRARY_PATH enviroment variable, or edit /etc/ld.so.conf\n");
        printf("*** so that the correct libraries are found at run-time))\n");
      }
    }
  return 1;
}
],, no_gtk=yes,[echo $ac_n "cross compiling; assumed OK... $ac_c"])
       CFLAGS="$ac_save_CFLAGS"
       LIBS="$ac_save_LIBS"
     fi
  fi
  if test "x$no_gtk" = x ; then
     AC_MSG_RESULT(yes)
     ifelse([$2], , :, [$2])     
  else
     AC_MSG_RESULT(no)
     if test "$GTK_CONFIG" = "no" ; then
       echo "*** The gtk-config script installed by GTK could not be found"
       echo "*** If GTK was installed in PREFIX, make sure PREFIX/bin is in"
       echo "*** your path, or set the GTK_CONFIG environment variable to the"
       echo "*** full path to gtk-config."
     else
       if test -f conf.gtktest ; then
        :
       else
          echo "*** Could not run GTK test program, checking why..."
          CFLAGS="$CFLAGS $GTK_CFLAGS"
          LIBS="$LIBS $GTK_LIBS"
          AC_TRY_LINK([
#include <gtk/gtk.h>
#include <stdio.h>
],      [ return ((gtk_major_version) || (gtk_minor_version) || (gtk_micro_version)); ],
        [ echo "*** The test program compiled, but did not run. This usually means"
          echo "*** that the run-time linker is not finding GTK or finding the wrong"
          echo "*** version of GTK. If it is not finding GTK, you'll need to set your"
          echo "*** LD_LIBRARY_PATH environment variable, or edit /etc/ld.so.conf to point"
          echo "*** to the installed location  Also, make sure you have run ldconfig if that"
          echo "*** is required on your system"
	  echo "***"
          echo "*** If you have an old version installed, it is best to remove it, although"
          echo "*** you may also be able to get things to work by modifying LD_LIBRARY_PATH"
          echo "***"
          echo "*** If you have a RedHat 5.0 system, you should remove the GTK package that"
          echo "*** came with the system with the command"
          echo "***"
          echo "***    rpm --erase --nodeps gtk gtk-devel" ],
        [ echo "*** The test program failed to compile or link. See the file config.log for the"
          echo "*** exact error that occured. This usually means GTK was incorrectly installed"
          echo "*** or that you have moved GTK since it was installed. In the latter case, you"
          echo "*** may want to edit the gtk-config script: $GTK_CONFIG" ])
          CFLAGS="$ac_save_CFLAGS"
          LIBS="$ac_save_LIBS"
       fi
     fi
     GTK_CFLAGS=""
     GTK_LIBS=""
     ifelse([$3], , :, [$3])
  fi
  AC_SUBST(GTK_CFLAGS)
  AC_SUBST(GTK_LIBS)
  rm -f conf.gtktest
])

dnl This M4 macro was found on:
dnl
dnl http://autoconf-archive.cryp.to/bnv_have_qt.m4
dnl
dnl @synopsis BNV_HAVE_QT [--with-Qt-dir=DIR] [--with-Qt-lib=LIB]
dnl @synopsis BNV_HAVE_QT [--with-Qt-include-dir=DIR] [--with-Qt-bin-dir=DIR] [--with-Qt-lib-dir=DIR] [--with-Qt-lib=LIB]
dnl
dnl @summary Search for Trolltech's Qt GUI framework.
dnl
dnl Searches common directories for Qt include files, libraries and Qt
dnl binary utilities. The macro supports several different versions of
dnl the Qt framework being installed on the same machine. Without
dnl options, the macro is designed to look for the latest library,
dnl i.e., the highest definition of QT_VERSION in qglobal.h. By use of
dnl one or more options a different library may be selected. There are
dnl two different sets of options. Both sets contain the option
dnl --with-Qt-lib=LIB which can be used to force the use of a
dnl particular version of the library file when more than one are
dnl available. LIB must be in the form as it would appear behind the
dnl "-l" option to the compiler. Examples for LIB would be "qt-mt" for
dnl the multi-threaded version and "qt" for the regular version. In
dnl addition to this, the first set consists of an option
dnl --with-Qt-dir=DIR which can be used when the installation conforms
dnl to Trolltech's standard installation, which means that header files
dnl are in DIR/include, binary utilities are in DIR/bin and the library
dnl is in DIR/lib. The second set of options can be used to indicate
dnl individual locations for the header files, the binary utilities and
dnl the library file, in addition to the specific version of the
dnl library file.
dnl
dnl The following shell variable is set to either "yes" or "no":
dnl
dnl   have_qt
dnl
dnl Additionally, the following variables are exported:
dnl
dnl   QT_CXXFLAGS
dnl   QT_LIBS
dnl   QT_MOC
dnl   QT_UIC
dnl   QT_DIR
dnl
dnl which respectively contain an "-I" flag pointing to the Qt include
dnl directory (and "-DQT_THREAD_SUPPORT" when LIB is "qt-mt"), link
dnl flags necessary to link with Qt and X, the name of the meta object
dnl compiler and the user interface compiler both with full path, and
dnl finaly the variable QTDIR as Trolltech likes to see it defined (if
dnl possible).
dnl
dnl Example lines for Makefile.in:
dnl
dnl   CXXFLAGS = @QT_CXXFLAGS@
dnl   MOC      = @QT_MOC@
dnl
dnl After the variables have been set, a trial compile and link is
dnl performed to check the correct functioning of the meta object
dnl compiler. This test may fail when the different detected elements
dnl stem from different releases of the Qt framework. In that case, an
dnl error message is emitted and configure stops.
dnl
dnl No common variables such as $LIBS or $CFLAGS are polluted.
dnl
dnl Options:
dnl
dnl --with-Qt-dir=DIR: DIR is equal to $QTDIR if you have followed the
dnl installation instructions of Trolltech. Header files are in
dnl DIR/include, binary utilities are in DIR/bin and the library is in
dnl DIR/lib.
dnl
dnl --with-Qt-include-dir=DIR: Qt header files are in DIR.
dnl
dnl --with-Qt-bin-dir=DIR: Qt utilities such as moc and uic are in DIR.
dnl
dnl --with-Qt-lib-dir=DIR: The Qt library is in DIR.
dnl
dnl --with-Qt-lib=LIB: Use -lLIB to link with the Qt library.
dnl
dnl If some option "=no" or, equivalently, a --without-Qt-* version is
dnl given in stead of a --with-Qt-*, "have_qt" is set to "no" and the
dnl other variables are set to the empty string.
dnl
dnl @category InstalledPackages
dnl @author Bastiaan Veelo <Bastiaan.N.Veelo@ntnu.no>
dnl @version 2005-01-24
dnl @license AllPermissive

dnl Copyright (C) 2001, 2002, 2003, 2005, Bastiaan Veelo

dnl Calls BNV_PATH_QT_DIRECT (contained in this file) as a subroutine.
AC_DEFUN([BNV_HAVE_QT],
[
  dnl THANKS! This code includes bug fixes and contributions made by:
  dnl Tim McClarren,
  dnl Dennis R. Weilert,
  dnl Qingning Huo.

  AC_REQUIRE([AC_PROG_CXX])
  AC_REQUIRE([AC_PATH_X])
  AC_REQUIRE([AC_PATH_XTRA])

  AC_MSG_CHECKING(for Qt)

  AC_ARG_WITH([Qt-dir],
    [  --with-Qt-dir=DIR       DIR is equal to \$QTDIR if you have followed the
                          installation instructions of Trolltech. Header
                          files are in DIR/include, binary utilities are
                          in DIR/bin and the library is in DIR/lib])
  AC_ARG_WITH([Qt-include-dir],
    [  --with-Qt-include-dir=DIR
                          Qt header files are in DIR])
  AC_ARG_WITH([Qt-bin-dir],
    [  --with-Qt-bin-dir=DIR   Qt utilities such as moc and uic are in DIR])
  AC_ARG_WITH([Qt-lib-dir],
    [  --with-Qt-lib-dir=DIR   The Qt library is in DIR])
  AC_ARG_WITH([Qt-lib],
    [  --with-Qt-lib=LIB       Use -lLIB to link with the Qt library])
  if test x"$with_Qt_dir" = x"no" ||
     test x"$with_Qt_include-dir" = x"no" ||
     test x"$with_Qt_bin_dir" = x"no" ||
     test x"$with_Qt_lib_dir" = x"no" ||
     test x"$with_Qt_lib" = x"no"; then
    # user disabled Qt. Leave cache alone.
    have_qt="User disabled Qt."
  else
    # "yes" is a bogus option
    if test x"$with_Qt_dir" = xyes; then
      with_Qt_dir=
    fi
    if test x"$with_Qt_include_dir" = xyes; then
      with_Qt_include_dir=
    fi
    if test x"$with_Qt_bin_dir" = xyes; then
      with_Qt_bin_dir=
    fi
    if test x"$with_Qt_lib_dir" = xyes; then
      with_Qt_lib_dir=
    fi
    if test x"$with_Qt_lib" = xyes; then
      with_Qt_lib=
    fi
    # No Qt unless we discover otherwise
    have_qt=no
    # Check whether we are requested to link with a specific version
    if test x"$with_Qt_lib" != x; then
      bnv_qt_lib="$with_Qt_lib"
    fi
    # Check whether we were supplied with an answer already
    if test x"$with_Qt_dir" != x; then
      have_qt=yes
      bnv_qt_dir="$with_Qt_dir"
      bnv_qt_include_dir="$with_Qt_dir/include"
      bnv_qt_bin_dir="$with_Qt_dir/bin"
      bnv_qt_lib_dir="$with_Qt_dir/lib"
      # Only search for the lib if the user did not define one already
      if test x"$bnv_qt_lib" = x; then
        bnv_qt_lib="`ls $bnv_qt_lib_dir/libqt* | sed -n 1p |
                     sed s@$bnv_qt_lib_dir/lib@@ | [sed s@[.].*@@]`"
      fi
      bnv_qt_LIBS="-L$bnv_qt_lib_dir -l$bnv_qt_lib $X_PRE_LIBS $X_LIBS -lX11 -lXext -lXmu -lXt -lXi $X_EXTRA_LIBS"
    else
      # Use cached value or do search, starting with suggestions from
      # the command line
      AC_CACHE_VAL(bnv_cv_have_qt,
      [
        # We are not given a solution and there is no cached value.
        bnv_qt_dir=NO
        bnv_qt_include_dir=NO
        bnv_qt_lib_dir=NO
        if test x"$bnv_qt_lib" = x; then
          bnv_qt_lib=NO
        fi
        BNV_PATH_QT_DIRECT
        if test "$bnv_qt_dir" = NO ||
           test "$bnv_qt_include_dir" = NO ||
           test "$bnv_qt_lib_dir" = NO ||
           test "$bnv_qt_lib" = NO; then
          # Problem with finding complete Qt.  Cache the known absence of Qt.
          bnv_cv_have_qt="have_qt=no"
        else
          # Record where we found Qt for the cache.
          bnv_cv_have_qt="have_qt=yes                  \
                       bnv_qt_dir=$bnv_qt_dir          \
               bnv_qt_include_dir=$bnv_qt_include_dir  \
                   bnv_qt_bin_dir=$bnv_qt_bin_dir      \
                      bnv_qt_LIBS=\"$bnv_qt_LIBS\""
        fi
      ])dnl
      eval "$bnv_cv_have_qt"
    fi # all $bnv_qt_* are set
  fi   # $have_qt reflects the system status
  if test x"$have_qt" = xyes; then
    QT_CXXFLAGS="-I$bnv_qt_include_dir"
    if test $bnv_qt_lib = "qt-mt"; then
        QT_CXXFLAGS="$QT_CXXFLAGS -DQT_THREAD_SUPPORT"
    fi
    QT_DIR="$bnv_qt_dir"
    QT_LIBS="$bnv_qt_LIBS"
    # If bnv_qt_dir is defined, utilities are expected to be in the
    # bin subdirectory
    if test x"$bnv_qt_dir" != x; then
        if test -x "$bnv_qt_dir/bin/uic"; then
          QT_UIC="$bnv_qt_dir/bin/uic"
        else
          # Old versions of Qt don't have uic
          QT_UIC=
        fi
      QT_MOC="$bnv_qt_dir/bin/moc"
    else
      # Or maybe we are told where to look for the utilities
      if test x"$bnv_qt_bin_dir" != x; then
        if test -x "$bnv_qt_bin_dir/uic"; then
          QT_UIC="$bnv_qt_bin_dir/uic"
        else
          # Old versions of Qt don't have uic
          QT_UIC=
        fi
        QT_MOC="$bnv_qt_bin_dir/moc"
      else
      # Last possibility is that they are in $PATH
        QT_UIC="`which uic`"
        QT_MOC="`which moc`"
      fi
    fi
    # All variables are defined, report the result
    AC_MSG_RESULT([$have_qt:
    QT_CXXFLAGS=$QT_CXXFLAGS
    QT_DIR=$QT_DIR
    QT_LIBS=$QT_LIBS
    QT_UIC=$QT_UIC
    QT_MOC=$QT_MOC])
  else
    # Qt was not found
    QT_CXXFLAGS=
    QT_DIR=
    QT_LIBS=
    QT_UIC=
    QT_MOC=
    AC_MSG_RESULT($have_qt)
  fi
  AC_SUBST(QT_CXXFLAGS)
  AC_SUBST(QT_DIR)
  AC_SUBST(QT_LIBS)
  AC_SUBST(QT_UIC)
  AC_SUBST(QT_MOC)

  #### Being paranoid:
  if test x"$have_qt" = xyes; then
    AC_MSG_CHECKING(correct functioning of Qt installation)
    AC_CACHE_VAL(bnv_cv_qt_test_result,
    [
      cat > bnv_qt_test.h << EOF
#include <qobject.h>
class Test : public QObject
{
Q_OBJECT
public:
  Test() {}
  ~Test() {}
public slots:
  void receive() {}
signals:
  void send();
};
EOF

      cat > bnv_qt_main.$ac_ext << EOF
#include "bnv_qt_test.h"
#include <qapplication.h>
int main( int argc, char **argv )
{
  QApplication app( argc, argv );
  Test t;
  QObject::connect( &t, SIGNAL(send()), &t, SLOT(receive()) );
}
EOF

      bnv_cv_qt_test_result="failure"
      bnv_try_1="$QT_MOC bnv_qt_test.h -o moc_bnv_qt_test.$ac_ext >/dev/null 2>bnv_qt_test_1.out"
      AC_TRY_EVAL(bnv_try_1)
      bnv_err_1=`grep -v '^ *+' bnv_qt_test_1.out | grep -v "^bnv_qt_test.h\$"`
      if test x"$bnv_err_1" != x; then
        echo "$bnv_err_1" >&AC_FD_CC
        echo "configure: could not run $QT_MOC on:" >&AC_FD_CC
        cat bnv_qt_test.h >&AC_FD_CC
      else
        bnv_try_2="$CXX $QT_CXXFLAGS -c $CXXFLAGS -o moc_bnv_qt_test.o moc_bnv_qt_test.$ac_ext >/dev/null 2>bnv_qt_test_2.out"
        AC_TRY_EVAL(bnv_try_2)
        bnv_err_2=`grep -v '^ *+' bnv_qt_test_2.out | grep -v "^bnv_qt_test.{$ac_ext}\$"`
        if test x"$bnv_err_2" != x; then
          echo "$bnv_err_2" >&AC_FD_CC
          echo "configure: could not compile:" >&AC_FD_CC
          cat bnv_qt_test.$ac_ext >&AC_FD_CC
        else
          bnv_try_3="$CXX $QT_CXXFLAGS -c $CXXFLAGS -o bnv_qt_main.o bnv_qt_main.$ac_ext >/dev/null 2>bnv_qt_test_3.out"
          AC_TRY_EVAL(bnv_try_3)
          bnv_err_3=`grep -v '^ *+' bnv_qt_test_3.out | grep -v "^bnv_qt_main.{$ac_ext}\$"`
          if test x"$bnv_err_3" != x; then
            echo "$bnv_err_3" >&AC_FD_CC
            echo "configure: could not compile:" >&AC_FD_CC
            cat bnv_qt_main.$ac_ext >&AC_FD_CC
          else
            bnv_try_4="$CXX $QT_LIBS $LIBS -o bnv_qt_main bnv_qt_main.o moc_bnv_qt_test.o >/dev/null 2>bnv_qt_test_4.out"
            AC_TRY_EVAL(bnv_try_4)
            bnv_err_4=`grep -v '^ *+' bnv_qt_test_4.out`
            if test x"$bnv_err_4" != x; then
              echo "$bnv_err_4" >&AC_FD_CC
            else
              bnv_cv_qt_test_result="succes"
            fi
          fi
        fi
      fi
    ])dnl AC_CACHE_VAL bnv_cv_qt_test_result
    AC_MSG_RESULT([$bnv_cv_qt_test_result]);
    if test x"$bnv_cv_qt_test_result" = "xfailure"; then
      AC_MSG_ERROR([Failed to find matching components of a complete
                  Qt installation. Try using more options,
                  see ./configure --help.])
    fi

    rm -f bnv_qt_test.h moc_bnv_qt_test.$ac_ext moc_bnv_qt_test.o \
          bnv_qt_main.$ac_ext bnv_qt_main.o bnv_qt_main \
          bnv_qt_test_1.out bnv_qt_test_2.out bnv_qt_test_3.out bnv_qt_test_4.out
  fi
])

dnl Internal subroutine of BNV_HAVE_QT
dnl Set bnv_qt_dir bnv_qt_include_dir bnv_qt_bin_dir bnv_qt_lib_dir bnv_qt_lib
AC_DEFUN(BNV_PATH_QT_DIRECT,
[
  ## Binary utilities ##
  if test x"$with_Qt_bin_dir" != x; then
    bnv_qt_bin_dir=$with_Qt_bin_dir
  fi
  ## Look for header files ##
  if test x"$with_Qt_include_dir" != x; then
    bnv_qt_include_dir="$with_Qt_include_dir"
  else
    # The following header file is expected to define QT_VERSION.
    qt_direct_test_header=qglobal.h
    # Look for the header file in a standard set of common directories.
    bnv_include_path_list="
      /usr/include
      `ls -dr /usr/include/qt* 2>/dev/null`
      `ls -dr /usr/lib/qt*/include 2>/dev/null`
      `ls -dr /usr/local/qt*/include 2>/dev/null`
      `ls -dr /opt/qt*/include 2>/dev/null`
    "
    for bnv_dir in $bnv_include_path_list; do
      if test -r "$bnv_dir/$qt_direct_test_header"; then
        bnv_dirs="$bnv_dirs $bnv_dir"
      fi
    done
    # Now look for the newest in this list
    bnv_prev_ver=0
    for bnv_dir in $bnv_dirs; do
      bnv_this_ver=`egrep -w '#define QT_VERSION' $bnv_dir/$qt_direct_test_header | sed s/'#define QT_VERSION'//`
      if expr $bnv_this_ver '>' $bnv_prev_ver > /dev/null; then
        bnv_qt_include_dir=$bnv_dir
        bnv_prev_ver=$bnv_this_ver
      fi
    done
  fi dnl Found header files.

  # Are these headers located in a traditional Trolltech installation?
  # That would be $bnv_qt_include_dir stripped from its last element:
  bnv_possible_qt_dir=`dirname $bnv_qt_include_dir`
  if test -x $bnv_possible_qt_dir/bin/moc &&
     ls $bnv_possible_qt_dir/lib/libqt* > /dev/null; then
    # Then the rest is a piece of cake
    bnv_qt_dir=$bnv_possible_qt_dir
    bnv_qt_bin_dir="$bnv_qt_dir/bin"
    ### Start patch Dennis Weilert
    #bnv_qt_lib_dir="$bnv_qt_dir/lib"
    if test x"$with_Qt_lib_dir" != x; then
      bnv_qt_lib_dir="$with_Qt_lib_dir"
    else
      bnv_qt_lib_dir="$bnv_qt_dir/lib"
    fi
    ### End patch Dennis Weilert
    # Only look for lib if the user did not supply it already
    if test x"$bnv_qt_lib" = xNO; then
      bnv_qt_lib="`ls $bnv_qt_lib_dir/libqt* | sed -n 1p |
                   sed s@$bnv_qt_lib_dir/lib@@ | [sed s@[.].*@@]`"
    fi
    bnv_qt_LIBS="-L$bnv_qt_lib_dir -l$bnv_qt_lib $X_PRE_LIBS $X_LIBS -lX11 -lXext -lXmu -lXt -lXi $X_EXTRA_LIBS"
  else
    # There is no valid definition for $QTDIR as Trolltech likes to see it
    bnv_qt_dir=
    ## Look for Qt library ##
    if test x"$with_Qt_lib_dir" != x; then
      bnv_qt_lib_dir="$with_Qt_lib_dir"
      # Only look for lib if the user did not supply it already
      if test x"$bnv_qt_lib" = xNO; then
        bnv_qt_lib="`ls $bnv_qt_lib_dir/libqt* | sed -n 1p |
                     sed s@$bnv_qt_lib_dir/lib@@ | [sed s@[.].*@@]`"
      fi
      bnv_qt_LIBS="-L$bnv_qt_lib_dir -l$bnv_qt_lib $X_PRE_LIBS $X_LIBS -lX11 -lXext -lXmu -lXt -lXi $X_EXTRA_LIBS"
    else
      # Normally, when there is no traditional Trolltech installation,
      # the library is installed in a place where the linker finds it
      # automatically.
      # If the user did not define the library name, try with qt
      if test x"$bnv_qt_lib" = xNO; then
        bnv_qt_lib=qt
      fi
      qt_direct_test_header=qapplication.h
      qt_direct_test_main="
        int argc;
        char ** argv;
        QApplication app(argc,argv);
      "
      # See if we find the library without any special options.
      # Don't add top $LIBS permanently yet
      bnv_save_LIBS="$LIBS"
      LIBS="-l$bnv_qt_lib $X_PRE_LIBS $X_LIBS -lX11 -lXext -lXmu -lXt -lXi $X_EXTRA_LIBS"
      bnv_qt_LIBS="$LIBS"
      bnv_save_CXXFLAGS="$CXXFLAGS"
      CXXFLAGS="-I$bnv_qt_include_dir"
      AC_TRY_LINK([#include <$qt_direct_test_header>],
        $qt_direct_test_main,
      [
        # Succes.
        # We can link with no special library directory.
        bnv_qt_lib_dir=
      ], [
        # That did not work. Try the multi-threaded version
        echo "Non-critical error, please neglect the above." >&AC_FD_CC
        bnv_qt_lib=qt-mt
        LIBS="-l$bnv_qt_lib $X_PRE_LIBS $X_LIBS -lX11 -lXext -lXmu -lXt -lXi $X_EXTRA_LIBS"
        AC_TRY_LINK([#include <$qt_direct_test_header>],
          $qt_direct_test_main,
        [
          # Succes.
          # We can link with no special library directory.
          bnv_qt_lib_dir=
        ], [
          # That did not work. Try the OpenGL version
          echo "Non-critical error, please neglect the above." >&AC_FD_CC
          bnv_qt_lib=qt-gl
          LIBS="-l$bnv_qt_lib $X_PRE_LIBS $X_LIBS -lX11 -lXext -lXmu -lXt -lXi $X_EXTRA_LIBS"
          AC_TRY_LINK([#include <$qt_direct_test_header>],
            $qt_direct_test_main,
          [
            # Succes.
            # We can link with no special library directory.
            bnv_qt_lib_dir=
          ], [
            # That did not work. Maybe a library version I don't know about?
            echo "Non-critical error, please neglect the above." >&AC_FD_CC
            # Look for some Qt lib in a standard set of common directories.
            bnv_dir_list="
              `echo $bnv_qt_includes | sed ss/includess`
              /lib
              /usr/lib
              /usr/local/lib
              /opt/lib
              `ls -dr /usr/lib/qt* 2>/dev/null`
              `ls -dr /usr/local/qt* 2>/dev/null`
              `ls -dr /opt/qt* 2>/dev/null`
            "
            for bnv_dir in $bnv_dir_list; do
              if ls $bnv_dir/libqt*; then
                # Gamble that it's the first one...
                bnv_qt_lib="`ls $bnv_dir/libqt* | sed -n 1p |
                            sed s@$bnv_dir/lib@@ | sed s/[.].*//`"
                bnv_qt_lib_dir="$bnv_dir"
                break
              fi
            done
            # Try with that one
            LIBS="-l$bnv_qt_lib $X_PRE_LIBS $X_LIBS -lX11 -lXext -lXmu -lXt -lXi $X_EXTRA_LIBS"
            AC_TRY_LINK([#include <$qt_direct_test_header>],
              $qt_direct_test_main,
            [
              # Succes.
              # We can link with no special library directory.
              bnv_qt_lib_dir=
            ], [
              # Leave bnv_qt_lib_dir defined
            ])
          ])
        ])
      ])
      if test x"$bnv_qt_lib_dir" != x; then
        bnv_qt_LIBS="-l$bnv_qt_lib_dir $LIBS"
      else
        bnv_qt_LIBS="$LIBS"
      fi
      LIBS="$bnv_save_LIBS"
      CXXFLAGS="$bnv_save_CXXFLAGS"
    fi dnl $with_Qt_lib_dir was not given
  fi dnl Done setting up for non-traditional Trolltech installation
])



# Usage:
#   SIM_AC_HAVE_COIN_IFELSE( IF-FOUND, IF-NOT-FOUND )
#
# Description:
#   This macro locates the Coin development system.  If it is found,
#   the set of variables listed below are set up as described and made
#   available to the configure script.
#
#   The $sim_ac_coin_desired variable can be set to false externally to
#   make Coin default to be excluded.
#
# Autoconf Variables:
# > $sim_ac_coin_desired     true | false (defaults to true)
# < $sim_ac_coin_avail       true | false
# < $sim_ac_coin_cppflags    (extra flags the preprocessor needs)
# < $sim_ac_coin_cflags      (extra flags the C compiler needs)
# < $sim_ac_coin_cxxflags    (extra flags the C++ compiler needs)
# < $sim_ac_coin_ldflags     (extra flags the linker needs)
# < $sim_ac_coin_libs        (link library flags the linker needs)
# < $sim_ac_coin_datadir     (location of Coin data files)
# < $sim_ac_coin_includedir  (location of Coin headers)
# < $sim_ac_coin_version     (the libCoin version)
# < $sim_ac_coin_msvcrt      (the MSVC++ C library Coin was built with)
# < $sim_ac_coin_configcmd   (the path to coin-config or "false")
#
# Authors:
#   Lars J. Aas, <larsa@sim.no>
#   Morten Eriksen, <mortene@sim.no>
#
# TODO:
#

AC_DEFUN([SIM_AC_HAVE_COIN_IFELSE], [
AC_PREREQ([2.14a])

# official variables
sim_ac_coin_avail=false
sim_ac_coin_cppflags=
sim_ac_coin_cflags=
sim_ac_coin_cxxflags=
sim_ac_coin_ldflags=
sim_ac_coin_libs=
sim_ac_coin_datadir=
sim_ac_coin_includedir=
sim_ac_coin_version=

# internal variables
: ${sim_ac_coin_desired=true}
sim_ac_coin_extrapath=

AC_ARG_WITH([coin],
AC_HELP_STRING([--with-coin], [enable use of Coin [[default=yes]]])
AC_HELP_STRING([--with-coin=DIR], [give prefix location of Coin]),
  [ case $withval in
    no)  sim_ac_coin_desired=false ;;
    yes) sim_ac_coin_desired=true ;;
    *)   sim_ac_coin_desired=true
         sim_ac_coin_extrapath=$withval ;;
    esac],
  [])

case $build in
*-mks ) sim_ac_pathsep=";" ;;
* )     sim_ac_pathsep="${PATH_SEPARATOR}" ;;
esac

if $sim_ac_coin_desired; then
  sim_ac_path=$PATH
  test -z "$sim_ac_coin_extrapath" || ## search in --with-coin path
    sim_ac_path="$sim_ac_coin_extrapath/bin${sim_ac_pathsep}$sim_ac_path"
  test x"$prefix" = xNONE ||          ## search in --prefix path
    sim_ac_path="$sim_ac_path${sim_ac_pathsep}$prefix/bin"

  AC_PATH_PROG(sim_ac_coin_configcmd, coin-config, false, $sim_ac_path)

  if ! test "X$sim_ac_coin_configcmd" = "Xfalse"; then
    test -n "$CONFIG" &&
      $sim_ac_coin_configcmd --alternate=$CONFIG >/dev/null 2>/dev/null &&
      sim_ac_coin_configcmd="$sim_ac_coin_configcmd --alternate=$CONFIG"
  fi

  if $sim_ac_coin_configcmd; then
    sim_ac_coin_version=`$sim_ac_coin_configcmd --version`
    sim_ac_coin_cppflags=`$sim_ac_coin_configcmd --cppflags`
    sim_ac_coin_cflags=`$sim_ac_coin_configcmd --cflags 2>/dev/null`
    sim_ac_coin_cxxflags=`$sim_ac_coin_configcmd --cxxflags`
    sim_ac_coin_ldflags=`$sim_ac_coin_configcmd --ldflags`
    sim_ac_coin_libs=`$sim_ac_coin_configcmd --libs`
    sim_ac_coin_datadir=`$sim_ac_coin_configcmd --datadir`
    # Hide stderr on the following, as ``--includedir'', ``--msvcrt''
    # and ``--cflags'' options were added late to coin-config.
    sim_ac_coin_includedir=`$sim_ac_coin_configcmd --includedir 2>/dev/null`
    sim_ac_coin_msvcrt=`$sim_ac_coin_configcmd --msvcrt 2>/dev/null`
    sim_ac_coin_cflags=`$sim_ac_coin_configcmd --cflags 2>/dev/null`
    AC_CACHE_CHECK(
      [if we can compile and link with the Coin library],
      sim_cv_coin_avail,
      [sim_ac_save_cppflags=$CPPFLAGS
      sim_ac_save_cxxflags=$CXXFLAGS
      sim_ac_save_ldflags=$LDFLAGS
      sim_ac_save_libs=$LIBS
      CPPFLAGS="$CPPFLAGS $sim_ac_coin_cppflags"
      CXXFLAGS="$CXXFLAGS $sim_ac_coin_cxxflags"
      LDFLAGS="$LDFLAGS $sim_ac_coin_ldflags"
      LIBS="$sim_ac_coin_libs $LIBS"
      AC_LANG_PUSH(C++)

      AC_TRY_LINK(
        [#include <Inventor/SoDB.h>],
        [SoDB::init();],
        [sim_cv_coin_avail=true],
        [sim_cv_coin_avail=false])

      AC_LANG_POP
      CPPFLAGS=$sim_ac_save_cppflags
      CXXFLAGS=$sim_ac_save_cxxflags
      LDFLAGS=$sim_ac_save_ldflags
      LIBS=$sim_ac_save_libs
    ])
    sim_ac_coin_avail=$sim_cv_coin_avail
  else # no 'coin-config' found
    locations=`IFS="${sim_ac_pathsep}"; for p in $sim_ac_path; do echo " -> $p/coin-config"; done`
    AC_MSG_WARN([cannot find 'coin-config' at any of these locations:
$locations])
    AC_MSG_WARN([
Need to be able to run 'coin-config' to figure out how to build and link
against the Coin library. To rectify this problem, you most likely need
to a) install Coin if it has not been installed, b) add the Coin install
bin/ directory to your PATH environment variable.
])
  fi
fi

if $sim_ac_coin_avail; then
  ifelse([$1], , :, [$1])
else
  ifelse([$2], , :, [$2])
fi
]) # SIM_AC_HAVE_COIN_IFELSE()



dnl Calls HAVE_FG_V4L2 (contained in this file) as a subroutine.
AC_DEFUN([HAVE_FG_V4L2],
[
#-----------------------------------------------------------------------
# check if video 4 Linux 2 is reachable
AC_MSG_CHECKING(whether Video For Linux Two api works)
AC_PREPROC_IFELSE([AC_LANG_SOURCE([[#include </usr/src/linux/include/linux/videodev2.h>]])],[have_include_videodev2=yes],[have_include_videodev2=no])

if test "x$have_include_videodev2" = "xyes"; then
  AC_DEFINE(VISP_HAVE_V4L2, 1, 
    [Define to 1 if the include linux/videodev2.h for Video4Linux2 acquisition is available])
  AC_MSG_RESULT(yes)
  CXXFLAGS="-I/usr/src/linux/include $CXXFLAGS"
else
  AC_MSG_RESULT(no)
fi
])

dnl Calls IRISA_HAVE_FG_ICCOMP (contained in this file) as a subroutine.
AC_DEFUN([IRISA_HAVE_FG_ICCOMP],
[
#-----------------------------------------------------------------------
# Manage the --with-fg-iccomp option
AC_ARG_WITH([fg-iccomp],
  AC_HELP_STRING([--with-fg-iccomp=DIR],
		 [Location of ICcomp framegrabber api. By default /udd/fspindle/robot/IC-comp/current]),
                 path_fg_iccomp=$withval, 
                 path_fg_iccomp=/udd/fspindle/robot/IC-comp/current)

# check if the IC-comp framegrabber home dir exists
#AC_CHECK_FILE([$path_fg_iccomp], [with_fg_iccomp=yes], [with_fg_iccomp=no])
with_fg_iccomp=yes
# set some variables if the IC-comp framegrabber is reacheable
AC_MSG_CHECKING(whether framegrabber ICcomp api works)
if test "x$with_fg_iccomp" = "xyes"; then
  FG_ICCOMP_HOME=$path_fg_iccomp
  FG_ICCOMP_DIR_INCLUDE=$FG_ICCOMP_HOME/include
  FG_ICCOMP_DIR_LIB=$FG_ICCOMP_HOME/lib
  FG_ICCOMP_CXXFLAGS=-I$FG_ICCOMP_DIR_INCLUDE
  FG_ICCOMP_LDFLAGS="-L$FG_ICCOMP_DIR_LIB"
  FG_ICCOMP_LIBS="-liccomp2x -lfl"
  # save values
  TMP_CXXFLAGS=$CXXFLAGS
  TMP_CPPFLAGS=$CPPFLAGS
  TMP_LDFLAGS=$LDFLAGS
  TMP_LIBS=$LIBS
  # set linker options for a local test 
  CXXFLAGS=$FG_ICCOMP_CXXFLAGS
  LDFLAGS=$FG_ICCOMP_LDFLAGS
  LIBS=$FG_ICCOMP_LIBS
  # test a single function of the SBS Vic2500 bus driver
  AC_LANG_PUSH(C++)		
  AC_LINK_IFELSE(
    [AC_LANG_PROGRAM([#include "ic-comp2x.h"],
                     [ICcomp2x fg;])],
    have_lib_iccomp2x=yes,
    have_lib_iccomp2x=no)
  AC_LANG_POP(C++)		
  # put the saved values
  CXXFLAGS=$TMP_CXXFLAGS
  CPPFLAGS=$TMP_CPPFLAGS
  LDFLAGS=$TMP_LDFLAGS
  LIBS=$TMP_LIBS

  if test "x$have_lib_iccomp2x" = "xyes"; then
    # Vic2500 robot works
    CXXFLAGS="$CXXFLAGS $FG_ICCOMP_CXXFLAGS"
    LDFLAGS="$LDFLAGS $FG_ICCOMP_LDFLAGS"
    LIBS="$FG_ICCOMP_LIBS $LIBS"
    AC_DEFINE([VISP_HAVE_ICCOMP], 1,
              [Define to 1 if the ICcomp framegrabber api is available])
    AC_MSG_RESULT(yes)
    AC_SUBST(FG_ICCOMP_CXXFLAGS)
    AC_SUBST(FG_ICCOMP_LDFLAGS)
    AC_SUBST(FG_ICCOMP_LIBS)
  else
    AC_MSG_RESULT(no)
  fi
else
  AC_MSG_RESULT(no)
fi

])
dnl Calls IRISA_HAVE_BIT3_DRIVER (contained in this file) as a subroutine.
AC_DEFUN([IRISA_HAVE_BIT3_DRIVER],
[

#-----------------------------------------------------------------------
# Manage Bit3 driver with --with-bit3-dir option
AC_ARG_WITH([bit3-dir],
  AC_HELP_STRING([--with-bit3=DIR],
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
    AC_DEFINE([VISP_HAVE_BIT3], 1,
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
dnl Calls IRISA_HAVE_ROBOT_AFMA4 (contained in this file) as a subroutine.
AC_DEFUN([IRISA_HAVE_ROBOT_AFMA4],
[

#-----------------------------------------------------------------------
# Manage Afma4 robot with --with-robot-Afma4 option
AC_ARG_WITH([robot-Afma4],
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
    [AC_LANG_PROGRAM(,)],
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
    AC_DEFINE([VISP_HAVE_AFMA4], 1,
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
dnl Calls IRISA_HAVE_ROBOT_AFMA6 (contained in this file) as a subroutine.
AC_DEFUN([IRISA_HAVE_ROBOT_AFMA6],
[
#-----------------------------------------------------------------------
# Manage Afma6 robot with --with-robot-Afma6 option
AC_ARG_WITH([robot-Afma6],
  AC_HELP_STRING([--with-robot-Afma6=DIR],
		 [Location of Afma6 robot api. By default /udd/fspindle/robot/Afma6/current]),
                 path_robot_afma6=$withval, 
                 path_robot_afma6=/udd/fspindle/robot/Afma6/current)
# check if the afma6 robot home dir exists
#AC_CHECK_FILE([$path_robot_afma6], [with_robot_afma6=yes], [with_robot_afma6=no])
with_robot_afma6=yes
AC_MSG_CHECKING(whether Afma6 robot api works)
# set some variables if the afma6 robot is reacheable
if test "x$with_robot_afma6" = "xyes"; then
  ROBOT_AFMA6_HOME=$path_robot_afma6
  ROBOT_AFMA6_DIR_INCLUDE=$ROBOT_AFMA6_HOME/include
  ROBOT_AFMA6_DIR_LIB=$ROBOT_AFMA6_HOME/lib
  ROBOT_AFMA6_CXXFLAGS=-I$ROBOT_AFMA6_DIR_INCLUDE
  ROBOT_AFMA6_LDFLAGS="-L$ROBOT_AFMA6_DIR_LIB"
  ROBOT_AFMA6_LIBS="-lrobotAfma6 -ltoolsAfma6 -luprimAfma6 -lbit3Afma6"
  # save values
  TMP_CXXFLAGS=$CXXFLAGS
  TMP_CPPFLAGS=$CPPFLAGS
  TMP_LDFLAGS=$LDFLAGS
  TMP_LIBS=$LIBS
  # set linker options for a local test 
  CXXFLAGS=$ROBOT_AFMA6_CXXFLAGS
  LDFLAGS=$ROBOT_AFMA6_LDFLAGS
  LIBS=$ROBOT_AFMA6_LIBS
  # test a single function of the Afma4 api
  AC_LANG_PUSH(C++)		
  AC_LINK_IFELSE(
    [AC_LANG_PROGRAM(,)],
    have_lib_toolsAfma6=yes,
    have_lib_toolsAfma6=no)
  AC_LANG_POP(C++)		
  # put the saved values
  CXXFLAGS=$TMP_CXXFLAGS
  CPPFLAGS=$TMP_CPPFLAGS
  LDFLAGS=$TMP_LDFLAGS
  LIBS=$TMP_LIBS

  if test "x$have_lib_toolsAfma6" = "xyes"; then
    # Afma6 robot works
    CXXFLAGS="$CXXFLAGS $ROBOT_AFMA6_CXXFLAGS"
    LDFLAGS="$LDFLAGS $ROBOT_AFMA6_LDFLAGS"
    LIBS="$ROBOT_AFMA6_LIBS $LIBS"
    AC_DEFINE([VISP_HAVE_AFMA6], 1,
              [Define to 1 if the Afma6 robot api are available])
    AC_MSG_RESULT(yes)
    AC_SUBST(ROBOT_AFMA6_CXXFLAGS)
    AC_SUBST(ROBOT_AFMA6_LDFLAGS)
    AC_SUBST(ROBOT_AFMA6_LIBS)
  else
    AC_MSG_RESULT(no)
  fi
else
  AC_MSG_RESULT(no)
fi


])
dnl Calls IRISA_HAVE_ROBOT_PTUEVI (contained in this file) as a subroutine.
AC_DEFUN([IRISA_HAVE_ROBOT_PTUEVI],
[

#-----------------------------------------------------------------------
# Manage the --with-robot-PtuEvi option
AC_ARG_WITH([robot-PtuEvi],
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
    [AC_LANG_PROGRAM([const char *name="foo"; int speed = 19200; char openserial(const char *, int );],
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
    AC_DEFINE([VISP_HAVE_PTUEVI], 1,
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
  LIBS=$ROBOT_BICLOPS_LIBS
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
    AC_DEFINE([VISP_HAVE_BICLOPS], 1,
              [Define to 1 if the biclops pan-tilt api is available])
    AC_MSG_RESULT(yes)
  else
    AC_MSG_RESULT(no)
  fi

])

