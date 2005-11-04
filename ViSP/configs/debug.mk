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
