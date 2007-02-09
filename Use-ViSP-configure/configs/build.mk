OS		= Linux
SUFFIX		= 
HOSTNAME	= rozetta.irisa.fr
PROJECT_HOME	= /local/soft/ViSP/Use-ViSP-configure-code
PROJECT_DIR_SRC	= $(PROJECT_HOME)/src
PROJECT_DIR_INCLUDE= $(PROJECT_HOME)/src
PROJECT_DIR_OBJ	= $(PROJECT_HOME)/.obj
PROJECT_DIR_DEP	= $(PROJECT_HOME)/.dep
PROJECT_LIB_PATH= $(PROJECT_HOME)/lib
PROJECT_BIN_PATH= $(PROJECT_HOME)/bin
PROJECT_OBJ_PATH= $(PROJECT_DIR_OBJ)/$(HOSTNAME)$(SUFFIX)
PROJECT_DEP_PATH= $(PROJECT_DIR_DEP)/$(HOSTNAME)$(SUFFIX)
PROJECT_LIBNAME	= example-1.0.0

HAVE_RTAI	                   = @HAVE_RTAI@
HAVE_MEAN_SPEED_CTRL_18KMH_SYNDEX6 = @HAVE_MEAN_SPEED_CTRL_18KMH_SYNDEX6@
HAVE_MEAN_SPEED_CTRL_30KMH_SYNDEX5 = @HAVE_MEAN_SPEED_CTRL_30KMH_SYNDEX5@
HAVE_SPEED_CTRL_30KMH_SYNDEX5      = @HAVE_SPEED_CTRL_30KMH_SYNDEX5@

CXX		= g++
CXX_VERSION	= 4.1.0
CXXFLAGS	=  -O2 -Wall -Wunused -Werror -I/local/soft/ViSP/Use-ViSP-configure-code/include -I.   -DUNIX -I/local/soft/ViSP/ViSP-build/include -I/usr/include -I/usr/local/include -I/usr/include/gtk-2.0 -I/usr/lib/glib-2.0/include -I/usr/include/glib-2.0 -I/usr/include/pango-1.0 -I/usr/lib/gtk-2.0/include -I/usr/include/atk-1.0 -I/usr/include/cairo -I/usr/include -I/usr/include -I/usr/include 
CPPFLAGS	= 
CXXALL		= $(CXX) $(CXXFLAGS) $(CPPFLAGS)
AR		= ar crs
RANLIB		= ranlib
MOC		= @QT_MOC@

LDFLAGS		=  -L/local/soft/ViSP/Use-ViSP-configure-code/lib
LIBS		= -lexample-1.0.0  -Wl,-rpath,/local/soft/ViSP/ViSP-build/lib -L/local/soft/ViSP/ViSP-build/lib -lvisp-2  -Wl,-rpath,/usr/local/lib -L/usr/local/lib -L/usr/lib -lSM -lICE -lSM -lICE -lX11 -lXext -lX11 -lXext -lm -lpthread  -lgsl -lgslcblas -lm -lgtk-x11-2.0 -lgdk-x11-2.0 -lglib-2.0 -lgobject-2.0 -lgmodule-2.0 -lgthread-2.0 -lraw1394 -ldc1394 


# --->
# ---> CFLAGS
# --->

# In debugging mode, (configure --enable-debug) we set automatically CXXFLAGS
# to -g -Wno-deprecated -DVP_DEBUG and turn off -O1, -O2, -O3 options

# LEVEL OF TRACE
ifeq ($(VP_DEBUG),yes)
	CPPFLAGS += -DVP_DEBUG=yes -g
	CXXFLAGST:=$(filter-out -O%,$(CXXFLAGS))
	CXXFLAGS=$(CXXFLAGST)
	# LEVEL OF TRACE
	ifdef VP_DMODE
	  CPPFLAGS += -DVP_DEBUG_MODE=$(VP_DMODE)
	else
	  # Niv de trace minimum par default.
	  CPPFLAGS += -DVP_DEBUG_MODE=0
	endif
endif


# FLAG DEFENSIF
ifeq ($(VP_DEFENSIF), yes)
  CPPFLAGS += -DVP_DEFENSIF
endif


# FLAG DEFENSIF
ifeq ($(VP_DEFENSIF), yes)
  CPPFLAGS += -DVP_DEFENSIF
endif

# --->
# ---> MAKE FLAGS
# --->

MAKE_FLAGS     =
ifeq ($(VP_DEBUG),yes)
  MAKE_FLAGS     += VP_DEBUG=yes
endif

ifeq ($(VP_DEFENSIF), yes)
  MAKE_FLAGS     += VP_DEFENSIF=yes
endif

ifdef VP_DMODE
  MAKE_FLAGS     += VP_DMODE=$(VP_DMODE)
endif
