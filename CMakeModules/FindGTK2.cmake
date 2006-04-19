##
## Copyright Projet Lagadic / IRISA-INRIA Rennes, 2006
## www: http://www.irisa.fr/lagadic
##
## Author: Fabien Spindler email:Fabien.Spindler@irisa.fr
##
## try to find GTK (and glib) and GTKGLArea
## Once run this will define: 
##
## GTK2_INCLUDE_DIR   - Directories to include to use GTK
## GTK2_LIBRARIES     - Files to link against to use GTK
## GTK2_FOUND         - If false, don't try to use GTK
## GTK2_GL_FOUND      - If false, don't try to use GTK's GL features

IF(UNIX)

  FIND_PATH( GTK2_gtk_INCLUDE_PATH gtk/gtk.h
    $ENV{GTK2_HOME}
    /usr/include/gtk-2.0
    /usr/local/include/gtk-2.0
    /opt/gnome/include/gtk-2.0
  )

  # Some Linux distributions (e.g. Red Hat) have glibconfig.h
  # and glib.h in different directories, so we need to look
  # for both.
  #  - Atanas Georgiev <atanas@cs.columbia.edu>

  FIND_PATH( GTK2_glibconfig_INCLUDE_PATH glibconfig.h
    /usr/lib/glib-2.0/include
    /opt/gnome/lib/glib-2.0/include
  )

  FIND_PATH( GTK2_glib_INCLUDE_PATH glib.h
    /usr/include/glib-2.0
    /opt/gnome/include/glib-2.0
  )

  FIND_PATH( GTK2_gtkgl_INCLUDE_PATH gtkgl/gtkglarea.h
    /usr/include
    /usr/local/include
    /usr/openwin/share/include
    /opt/gnome/include
  )

  FIND_PATH( GTK2_pango_INCLUDE_PATH pango/pango.h
    /usr/include/pango-1.0
    /opt/gnome/include/pango-1.0
  )

  FIND_PATH( GTK2_gdkconfig_INCLUDE_PATH gdkconfig.h
    /usr/lib/gtk-2.0/include
    /opt/gnome/lib/gtk-2.0/include
  )

  FIND_PATH( GTK2_atk_INCLUDE_PATH atk/atk.h
    /usr/include/atk-1.0
    /opt/gnome/include/atk-1.0
  )


  FIND_LIBRARY( GTK2_gtkgl_LIBRARY gtkgl
    /usr/lib
    /usr/local/lib
    /usr/openwin/lib
    /usr/X11R6/lib
    /opt/gnome/lib
  )

  #
  # The 12 suffix is thanks to the FreeBSD ports collection
  #

  FIND_LIBRARY( GTK2_gtk_LIBRARY
    NAMES  gtk-x11-2.0 
    PATHS /usr/lib
          /usr/local/lib
          /usr/openwin/lib
          /usr/X11R6/lib
          /opt/gnome/lib
  )

  FIND_LIBRARY( GTK2_gdk_LIBRARY
    NAMES  gdk-x11-2.0
    PATHS  /usr/lib
           /usr/local/lib
           /usr/openwin/lib
           /usr/X11R6/lib
           /opt/gnome/lib
  )

  FIND_LIBRARY( GTK2_gmodule_LIBRARY
    NAMES  gmodule-2.0
    PATHS  /usr/lib
           /usr/local/lib
           /usr/openwin/lib
           /usr/X11R6/lib
           /opt/gnome/lib
  )

  FIND_LIBRARY( GTK2_glib_LIBRARY
    NAMES  glib-2.0
    PATHS  /usr/lib
           /usr/local/lib
           /usr/openwin/lib
           /usr/X11R6/lib
           /opt/gnome/lib
  )

  FIND_LIBRARY( GTK2_Xi_LIBRARY 
    NAMES Xi 
    PATHS /usr/lib 
    /usr/local/lib 
    /usr/openwin/lib 
    /usr/X11R6/lib 
    /opt/gnome/lib 
    ) 

  FIND_LIBRARY( GTK2_gthread_LIBRARY
    NAMES  gthread-2.0
    PATHS  /usr/lib
           /usr/local/lib
           /usr/openwin/lib
           /usr/X11R6/lib
           /opt/gnome/lib
  )

  FIND_LIBRARY( GTK2_gobject_LIBRARY
    NAMES  gobject-2.0
    PATHS 
           /usr/lib
           /opt/gnome/lib
  )

  IF(GTK2_gtk_INCLUDE_PATH)
  IF(GTK2_glibconfig_INCLUDE_PATH)
  IF(GTK2_glib_INCLUDE_PATH)
  IF(GTK2_gtk_LIBRARY)
  IF(GTK2_glib_LIBRARY)
  IF(GTK2_pango_INCLUDE_PATH)
    IF(GTK2_atk_INCLUDE_PATH)
    # Assume that if gtk and glib were found, the other
    # supporting libraries have also been found.

    SET( GTK2_FOUND "YES" )
    SET( GTK2_INCLUDE_DIR  ${GTK2_gtk_INCLUDE_PATH}
                           ${GTK2_glibconfig_INCLUDE_PATH}
                           ${GTK2_glib_INCLUDE_PATH} 
			   ${GTK2_pango_INCLUDE_PATH}
			   ${GTK2_gdkconfig_INCLUDE_PATH}
			   ${GTK2_atk_INCLUDE_PATH})
    SET( GTK2_LIBRARIES  ${GTK2_gtk_LIBRARY}
                        ${GTK2_gdk_LIBRARY}
                        ${GTK2_glib_LIBRARY} 
			${GTK2_gobject_LIBRARY})

    IF(GTK2_gmodule_LIBRARY)
      SET(GTK2_LIBRARIES ${GTK2_LIBRARIES} ${GTK2_gmodule_LIBRARY})
    ENDIF(GTK2_gmodule_LIBRARY)
    IF(GTK2_gthread_LIBRARY)
      SET(GTK2_LIBRARIES ${GTK2_LIBRARIES} ${GTK2_gthread_LIBRARY})
    ENDIF(GTK2_gthread_LIBRARY)

 ELSE(GTK2_atk_INCLUDE_PATH)
   MESSAGE("Can not find atk")
 ENDIF(GTK2_atk_INCLUDE_PATH)

  ELSE(GTK2_pango_INCLUDE_PATH)
       #MESSAGE("Can not find pango includes")
  ENDIF(GTK2_pango_INCLUDE_PATH)
  ELSE(GTK2_glib_LIBRARY)
       #MESSAGE("Can not find glib lib")
  ENDIF(GTK2_glib_LIBRARY)
  ELSE(GTK2_gtk_LIBRARY)
       #MESSAGE("Can not find gtk lib")
  ENDIF(GTK2_gtk_LIBRARY)
  ELSE(GTK2_glib_INCLUDE_PATH) 
   #MESSAGE("Can not find glib includes")
  ENDIF(GTK2_glib_INCLUDE_PATH) 
  ELSE(GTK2_glibconfig_INCLUDE_PATH)
   #MESSAGE("Can not find glibconfig")
  ENDIF(GTK2_glibconfig_INCLUDE_PATH)
  ELSE(GTK2_gtk_INCLUDE_PATH)
   #MESSAGE("Can not find gtk includes")
  ENDIF(GTK2_gtk_INCLUDE_PATH)

  MARK_AS_ADVANCED(
    GTK2_gdk_LIBRARY
    GTK2_glib_INCLUDE_PATH
    GTK2_glib_LIBRARY
    GTK2_glibconfig_INCLUDE_PATH
    GTK2_gmodule_LIBRARY
    GTK2_gthread_LIBRARY
    GTK2_Xi_LIBRARY
    GTK2_gtk_INCLUDE_PATH
    GTK2_gtk_LIBRARY
    GTK2_gtkgl_INCLUDE_PATH
    GTK2_gtkgl_LIBRARY
    GTK2_atk_INCLUDE_PATH
    GTK2_gdkconfig_INCLUDE_PATH
    GTK2_gobject_LIBRARY
    GTK2_pango_INCLUDE_PATH 
  )

ELSE(UNIX)
  # MESSAGE("FindGTK2 is working on UNIX/LINUX, only!")
ENDIF(UNIX)

