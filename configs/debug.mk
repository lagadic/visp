# --->
# ---> CFLAGS
# --->

# In debugging mode, (configure --enable-debug) we set automatically CXXFLAGS
# to -g -Wno-deprecated -DVP_DEBUG and turn off -O1, -O2, -O3 options

# LEVEL OF TRACE
ifdef VP_DEBUG
ifdef DMODE
  CPPFLAGS += -DVP_DEBUG_MODE=$(DMODE)
else
  # Niv de trace minimum par default.
  CPPFLAGS += -DVP_DEBUG_MODE=0
endif
endif

# FLAG DEFENSIF
ifeq ($(DEFENSIF), yes)
  CPPFLAGS += -DDEFENSIF
endif

# --->
# ---> MAKE FLAGS
# --->

MAKE_FLAGS     =
ifeq ($(VP_DEBUG),yes)
  MAKE_FLAGS     += DEBUG=yes
endif

ifeq ($(DEFENSIF), yes)
  MAKE_FLAGS     += DEFENSIF=yes
endif

ifdef DMODE
  MAKE_FLAGS     += DMODE=$(DMODE)
endif
