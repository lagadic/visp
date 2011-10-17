#############################################################################
#
# $Id$
#
# This file is part of the ViSP software.
# Copyright (C) 2005 - 2011 by INRIA. All rights reserved.
# 
# This software is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# ("GPL") version 2 as published by the Free Software Foundation.
# See the file LICENSE.txt at the root directory of this source
# distribution for additional information about the GNU GPL.
#
# For using ViSP with software that can not be combined with the GNU
# GPL, please contact INRIA about acquiring a ViSP Professional 
# Edition License.
#
# See http://www.irisa.fr/lagadic/visp/visp.html for more information.
# 
# This software was developed at:
# INRIA Rennes - Bretagne Atlantique
# Campus Universitaire de Beaulieu
# 35042 Rennes Cedex
# France
# http://www.irisa.fr/lagadic
#
# If you have questions regarding the use of this file, please contact
# INRIA at visp@inria.fr
# 
# This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
# WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
#
# Description:
# Try to find GTK (and glib) and GTKGLArea.
# Once run this will define: 
#
# GTK_INCLUDE_DIRS  - Directories to include to use GTK
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
    SET( GTK_INCLUDE_DIRS  ${GTK_gtk_INCLUDE_PATH}
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
    LIST(APPEND GTK_INCLUDE_DIRS ${GTK_gtkgl_INCLUDE_PATH} )
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
    GTK_pango_INCLUDE_PATH
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

