#############################################################################
#
# ViSP, open source Visual Servoing Platform software.
# Copyright (C) 2005 - 2023 by Inria. All rights reserved.
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
# See https://visp.inria.fr for more information.
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
# Try to find GTK (and glib).
# Once run this will define:
#
# GTK2_INCLUDE_DIRS  - Directories to include to use GTK
# GTK2_LIBRARIES     - Files to link against to use GTK
# GTK2_FOUND         - If false, don't try to use GTK
# GTK2_GL_FOUND      - If false, don't try to use GTK's GL features
# GTK2_VERSION
#
#############################################################################

if(UNIX OR WIN32)

  find_path( GTK2_gtk_INCLUDE_PATH gtk/gtk.h
    $ENV{GTK2_DIR}/include/gtk-2.0
    $ENV{GTK2_HOME}/include/gtk-2.0
    /usr/include/gtk-2.0
    /usr/local/include/gtk-2.0
    /opt/gnome/include/gtk-2.0
    "C:/GTK/include/gtk-2.0"
    /sw/include/gtk-2.0
  )

  # Some Linux distributions (e.g. Red Hat) have glibconfig.h
  # and glib.h in different directories, so we need to look
  # for both.
  #  - Atanas Georgiev <atanas@cs.columbia.edu>

  find_path( GTK2_glibconfig_INCLUDE_PATH glibconfig.h
    $ENV{GTK2_DIR}/lib/glib-2.0/include
    $ENV{GTK2_HOME}/lib/glib-2.0/include
    /usr/lib/glib-2.0/include
    /usr/lib64/glib-2.0/include
    /usr/lib/i386-linux-gnu/glib-2.0/include
    /usr/lib/x86_64-linux-gnu/glib-2.0/include
    /opt/gnome/lib/glib-2.0/include
    C:/GTK/lib/glib-2.0/include
    /sw/lib/glib-2.0/include
  )

  find_path( GTK2_glib_INCLUDE_PATH glib.h
    $ENV{GTK2_DIR}/include/glib-2.0
    $ENV{GTK2_HOME}/include/glib-2.0
    /usr/include/glib-2.0
    /opt/gnome/include/glib-2.0
    C:/GTK/include/glib-2.0
    /sw/include/glib-2.0
  )

  find_path( GTK2_pango_INCLUDE_PATH pango/pango.h
    $ENV{GTK2_DIR}/include/pango-1.0
    $ENV{GTK2_HOME}/include/pango-1.0
    /usr/include/pango-1.0
    /opt/gnome/include/pango-1.0
    C:/GTK/include/pango-1.0
    /sw/include/pango-1.0
  )

  find_path( GTK2_harfbuzz_INCLUDE_PATH hb.h
    $ENV{GTK2_DIR}/include/harfbuzz
    $ENV{GTK2_HOME}/include/harfbuzz
    /usr/include/harfbuzz
    /opt/gnome/include/harfbuzz
    C:/GTK/include/harfbuzz
    /sw/include/harfbuzz
  )

  find_path( GTK2_cairo_INCLUDE_PATH cairo.h
    $ENV{GTK2_DIR}/include/cairo
    $ENV{GTK2_HOME}/include/cairo
    /usr/include/cairo
    /opt/gnome/include/cairo
    C:/GTK/include/cairo
    /sw/include/cairo
  )

  find_path( GTK2_gdkconfig_INCLUDE_PATH gdkconfig.h
    $ENV{GTK2_DIR}/lib/gtk-2.0/include
    $ENV{GTK2_HOME}/lib/gtk-2.0/include
    /usr/lib/gtk-2.0/include
    /usr/lib64/gtk-2.0/include
    /opt/gnome/lib/gtk-2.0/include
    C:/GTK/lib/gtk-2.0/include
    /sw/lib/gtk-2.0/include
    /usr/lib/i386-linux-gnu/gtk-2.0/include
    /usr/lib/x86_64-linux-gnu/gtk-2.0/include
  )

  find_path( GTK2_gdkpixbuf_INCLUDE_PATH gdk-pixbuf/gdk-pixbuf.h
    $ENV{GTK2_DIR}/gdk-pixbuf-2.0
    $ENV{GTK2_HOME}/gdk-pixbuf-2.0
    /usr/include/gdk-pixbuf-2.0
    /usr/include/gtk-2.0
    C:/GTK/include/gtk-2.0
    /sw/include/gtk-2.0
  )

  find_path( GTK2_atk_INCLUDE_PATH atk/atk.h
    $ENV{GTK2_DIR}/include/atk-1.0
    $ENV{GTK2_HOME}/include/atk-1.0
    /usr/include/atk-1.0
    /opt/gnome/include/atk-1.0
    C:/GTK/include/atk-1.0
    /sw/include/atk-1.0
  )

  find_library( GTK2_gtk_LIBRARY
    NAMES  gtk-x11-2.0 gtk-win32-2.0
    PATHS $ENV{GTK2_DIR}/lib
    PATHS $ENV{GTK2_HOME}/lib
          /usr/lib
          /usr/local/lib
          /usr/openwin/lib
          /usr/X11R6/lib
          /opt/gnome/lib
          C:/GTK/lib
	  /sw/lib
  )

  find_library( GTK2_gdk_LIBRARY
    NAMES  gdk-x11-2.0 gdk-win32-2.0
    PATHS  $ENV{GTK2_DIR}/lib
    PATHS  $ENV{GTK2_HOME}/lib
           /usr/lib
           /usr/local/lib
           /usr/openwin/lib
           /usr/X11R6/lib
           /opt/gnome/lib
           C:/GTK/lib
	   /sw/lib
  )

  find_library( GTK2_gmodule_LIBRARY
    NAMES  gmodule-2.0
    PATHS  $ENV{GTK2_DIR}/lib
    PATHS  $ENV{GTK2_HOME}/lib
           /usr/lib
           /usr/local/lib
           /usr/openwin/lib
           /usr/X11R6/lib
           /opt/gnome/lib
           C:/GTK/lib
	   /sw/lib
  )

  find_library( GTK2_glib_LIBRARY
    NAMES  glib-2.0
    PATHS  $ENV{GTK2_DIR}/lib
    PATHS  $ENV{GTK2_HOME}/lib
           /usr/lib
           /usr/local/lib
           /usr/openwin/lib
           /usr/X11R6/lib
           /opt/gnome/lib
           C:/GTK/lib
	   /sw/lib
  )

  find_library( GTK2_gthread_LIBRARY
    NAMES  gthread-2.0
    PATHS  $ENV{GTK2_DIR}/lib
    PATHS  $ENV{GTK2_HOME}/lib
           /usr/lib
           /usr/local/lib
           /usr/openwin/lib
           /usr/X11R6/lib
           /opt/gnome/lib
           C:/GTK/lib
	   /sw/lib
  )

  find_library( GTK2_gobject_LIBRARY
    NAMES  gobject-2.0
    PATHS  $ENV{GTK2_DIR}/lib
    PATHS  $ENV{GTK2_HOME}/lib
           /usr/lib
           /opt/gnome/lib
           C:/GTK/lib
	   /sw/lib
  )

  if(GTK2_gtk_INCLUDE_PATH)
  if(GTK2_glib_INCLUDE_PATH)
  if(GTK2_glibconfig_INCLUDE_PATH)
  if(GTK2_gtk_LIBRARY)
  if(GTK2_glib_LIBRARY)
  if(GTK2_pango_INCLUDE_PATH)
  if(GTK2_gdkconfig_INCLUDE_PATH)
    if(GTK2_atk_INCLUDE_PATH)
      # Assume that if gtk and glib were found, the other
      # supporting libraries have also been found.

      set( GTK2_FOUND "YES" )
      set( GTK2_INCLUDE_DIRS ${GTK2_gtk_INCLUDE_PATH}
                             ${GTK2_glib_INCLUDE_PATH}
                             ${GTK2_glibconfig_INCLUDE_PATH}
                             ${GTK2_pango_INCLUDE_PATH}
                             ${GTK2_atk_INCLUDE_PATH}
                             ${GTK2_gdkconfig_INCLUDE_PATH})
      if(GTK2_cairo_INCLUDE_PATH)
        list(APPEND GTK2_INCLUDE_DIRS  ${GTK2_cairo_INCLUDE_PATH} )
      endif()
      if(GTK2_gdkpixbuf_INCLUDE_PATH)
        LIST(APPEND GTK2_INCLUDE_DIRS  ${GTK2_gdkpixbuf_INCLUDE_PATH} )
      endif()
      if(GTK2_gdkconfig_INCLUDE_PATH)
        LIST(APPEND GTK2_INCLUDE_DIRS  ${GTK2_gdkconfig_INCLUDE_PATH} )
      endif()
      if(GTK2_harfbuzz_INCLUDE_PATH)
        LIST(APPEND GTK2_INCLUDE_DIRS  ${GTK2_harfbuzz_INCLUDE_PATH} )
      endif()

      set( GTK2_LIBRARIES ${GTK2_gtk_LIBRARY}
                          ${GTK2_gdk_LIBRARY}
                          ${GTK2_glib_LIBRARY}
                          ${GTK2_gobject_LIBRARY})

      get_filename_component(GTK2_LIB_DIR ${GTK2_gtk_LIBRARY} PATH)
      vp_get_version_from_pkg("gtk+-2.0" "${GTK2_LIB_DIR}/pkgconfig" GTK2_VERSION)

      if(GTK2_gmodule_LIBRARY)
        list(APPEND GTK2_LIBRARIES ${GTK2_gmodule_LIBRARY})
      endif()
      if(GTK2_gthread_LIBRARY)
        list(APPEND GTK2_LIBRARIES ${GTK2_gthread_LIBRARY})
      endif()

    else(GTK2_atk_INCLUDE_PATH)
      message("Can not find atk")
    endif()
  else(GTK2_gdkconfig_INCLUDE_PATH)
    #message("Can not find gdkconfig include")
  endif(GTK2_gdkconfig_INCLUDE_PATH)
  else(GTK2_pango_INCLUDE_PATH)
    #message("Can not find pango includes")
  endif(GTK2_pango_INCLUDE_PATH)
  else(GTK2_glib_LIBRARY)
    #message("Can not find glib lib")
  endif(GTK2_glib_LIBRARY)
  else(GTK2_gtk_LIBRARY)
    #message("Can not find gtk lib")
  endif(GTK2_gtk_LIBRARY)
  else(GTK2_glibconfig_INCLUDE_PATH)
    #message("Can not find glibconfig includes")
  endif(GTK2_glibconfig_INCLUDE_PATH)
  else(GTK2_glib_INCLUDE_PATH)
    #message("Can not find glib includes")
  endif(GTK2_glib_INCLUDE_PATH)
  else(GTK2_gtk_INCLUDE_PATH)
    #message("Can not find gtk includes")
  endif(GTK2_gtk_INCLUDE_PATH)

  mark_as_advanced(
    GTK2_gdk_LIBRARY
    GTK2_glib_INCLUDE_PATH
    GTK2_glib_LIBRARY
    GTK2_glibconfig_INCLUDE_PATH
    GTK2_gmodule_LIBRARY
    GTK2_gthread_LIBRARY
    GTK2_gtk_INCLUDE_PATH
    GTK2_gtk_LIBRARY
    GTK2_atk_INCLUDE_PATH
    GTK2_gdkconfig_INCLUDE_PATH
    GTK2_gobject_LIBRARY
    GTK2_pango_INCLUDE_PATH
    GTK2_cairo_INCLUDE_PATH
    GTK2_harfbuzz_INCLUDE_PATH
    GTK2_gdkpixbuf_INCLUDE_PATH
  )

else(UNIX OR WIN32)
  message("FindGTK2 is working on UNIX/LINUX and Windows, only!")
endif()
