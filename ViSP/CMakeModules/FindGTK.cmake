#############################################################################
#
# $Id$
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
# Try to find GTK (and glib) and GTKGLArea.
# Once run this will define: 
#
# GTK_INCLUDE_DIR   - Directories to include to use GTK
# GTK_LIBRARIES     - Files to link against to use GTK
# GTK_FOUND         - If false, don't try to use GTK
# GTK_GL_FOUND      - If false, don't try to use GTK's GL features
#
# Authors:
# Fabien Spindler
#
#############################################################################

# don't even bother under WIN32
IF(UNIX)

  FIND_PATH( GTK_gtk_INCLUDE_PATH gtk/gtk.h
    $ENV{GTK_HOME}
    /usr/include
    /usr/local/include
    /usr/openwin/share/include
    /usr/openwin/include
    /usr/X11R6/include
    /usr/include/X11
    /usr/X11R6/include/gtk12
    /usr/include/gtk-1.2
    /usr/local/include/gtk-1.2
    /opt/gnome/include
    /opt/gnome/include/gtk-1.2
    /sw/include
    /sw/include/gtk-1.2
  )

  # Some Linux distributions (e.g. Red Hat) have glibconfig.h
  # and glib.h in different directories, so we need to look
  # for both.
  #  - Atanas Georgiev <atanas@cs.columbia.edu>

  FIND_PATH( GTK_glibconfig_INCLUDE_PATH glibconfig.h
    /usr/include
    /usr/local/include
    /usr/openwin/share/include
    /usr/local/include/glib12
    /usr/lib/glib/include
    /usr/local/lib/glib/include
    /opt/gnome/include
    /opt/gnome/lib/glib/include
    /sw/lib/glib/include 
  )

  FIND_PATH( GTK_glib_INCLUDE_PATH glib.h
    /usr/include
    /usr/local/include
    /usr/openwin/share/include
    /usr/include/gtk-1.2
    /usr/local/include/glib12
    /usr/lib/glib/include
    /usr/include/glib-1.2
    /usr/local/include/glib-1.2
    /opt/gnome/include
    /opt/gnome/include/glib-1.2
    /sw/include	
    /sw/include/glib-1.2 
)


  FIND_PATH( GTK_pango_INCLUDE_PATH pango/pango.h
  /usr/include/pango-1.0
  /opt/gnome/include/pango-1.0
  )

  #
  # The 12 suffix is thanks to the FreeBSD ports collection
  #

  FIND_LIBRARY( GTK_gtk_LIBRARY
    NAMES  gtk gtk12
    PATHS /usr/lib
          /usr/local/lib
          /usr/openwin/lib
          /usr/X11R6/lib
          /opt/gnome/lib
	  /sw/lib
  )

  FIND_LIBRARY( GTK_gdk_LIBRARY
    NAMES  gdk gdk12
    PATHS  /usr/lib
           /usr/local/lib
           /usr/openwin/lib
           /usr/X11R6/lib
           /opt/gnome/lib
	   /sw/lib
  )

  FIND_LIBRARY( GTK_gmodule_LIBRARY
    NAMES  gmodule gmodule12
    PATHS  /usr/lib
           /usr/local/lib
           /usr/openwin/lib
           /usr/X11R6/lib
           /opt/gnome/lib
 	   /sw/lib
 )

  FIND_LIBRARY( GTK_glib_LIBRARY
    NAMES  glib glib12
    PATHS  /usr/lib
           /usr/local/lib
           /usr/openwin/lib
           /usr/X11R6/lib
           /opt/gnome/lib
  	   /sw/lib
)

  FIND_LIBRARY( GTK_Xi_LIBRARY 
    NAMES Xi 
    PATHS /usr/lib 
    /usr/local/lib 
    /usr/openwin/lib 
    /usr/X11R6/lib 
    /opt/gnome/lib 
    /sw/lib
  ) 

  FIND_LIBRARY( GTK_gthread_LIBRARY
    NAMES  gthread gthread12
    PATHS  /usr/lib
           /usr/local/lib
           /usr/openwin/lib
           /usr/X11R6/lib
           /opt/gnome/lib
  	   /sw/lib
)

  IF(GTK_gtk_INCLUDE_PATH)
  IF(GTK_glibconfig_INCLUDE_PATH)
  IF(GTK_glib_INCLUDE_PATH)
  IF(GTK_pango_INCLUDE_PATH)
  IF(GTK_gtk_LIBRARY)
  IF(GTK_glib_LIBRARY)

    # Assume that if gtk and glib were found, the other
    # supporting libraries have also been found.

    SET( GTK_FOUND "YES" )
    SET( GTK_INCLUDE_DIR  ${GTK_gtk_INCLUDE_PATH}
                           ${GTK_glibconfig_INCLUDE_PATH}
                           ${GTK_glib_INCLUDE_PATH} )
    SET( GTK_LIBRARIES  ${GTK_gtk_LIBRARY}
                        ${GTK_gdk_LIBRARY}
                        ${GTK_glib_LIBRARY} )

    IF(GTK_gmodule_LIBRARY)
      SET(GTK_LIBRARIES ${GTK_LIBRARIES} ${GTK_gmodule_LIBRARY})
    ENDIF(GTK_gmodule_LIBRARY)
    IF(GTK_gthread_LIBRARY)
      SET(GTK_LIBRARIES ${GTK_LIBRARIES} ${GTK_gthread_LIBRARY})
    ENDIF(GTK_gthread_LIBRARY)
    IF(GTK_Xi_LIBRARY)
      SET(GTK_LIBRARIES ${GTK_LIBRARIES} ${GTK_Xi_LIBRARY})
    ENDIF(GTK_Xi_LIBRARY)

  IF(GTK_gtkgl_INCLUDE_PATH)
  IF(GTK_gtkgl_LIBRARY)
    SET( GTK_GL_FOUND "YES" )
    SET( GTK_INCLUDE_DIR  ${GTK_INCLUDE_DIR}
                           ${GTK_gtkgl_INCLUDE_PATH} )
    SET( GTK_LIBRARIES  ${GTK_gtkgl_LIBRARY} ${GTK_LIBRARIES} )
    MARK_AS_ADVANCED(
      GTK_gtkgl_LIBRARY
      GTK_gtkgl_INCLUDE_PATH
      )
  ENDIF(GTK_gtkgl_LIBRARY)
  ENDIF(GTK_gtkgl_INCLUDE_PATH)

  ENDIF(GTK_glib_LIBRARY)
  ENDIF(GTK_gtk_LIBRARY)
  ENDIF(GTK_pango_INCLUDE_PATH)
  ENDIF(GTK_glib_INCLUDE_PATH) 
  ENDIF(GTK_glibconfig_INCLUDE_PATH)
  ENDIF(GTK_gtk_INCLUDE_PATH)

  MARK_AS_ADVANCED(
    GTK_gdk_LIBRARY
    GTK_glib_INCLUDE_PATH
    GTK_glib_LIBRARY
    GTK_glibconfig_INCLUDE_PATH
    GTK_gmodule_LIBRARY
    GTK_gthread_LIBRARY
    GTK_Xi_LIBRARY
    GTK_gtk_INCLUDE_PATH
    GTK_gtk_LIBRARY
    GTK_gtkgl_INCLUDE_PATH
    GTK_gtkgl_LIBRARY
    )

ELSE(UNIX)
  # MESSAGE("FindGTK is working on UNIX/LINUX, only!")
ENDIF(UNIX)

