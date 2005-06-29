# --->
# ---> CFLAGS
# --->

# In debugging mode, (configure --enable-debug) we set automatically CXXFLAGS
# to -g -Wno-deprecated -DDEBUG and turn off -O1, -O2, -O3 options

# LEVEL OF TRACE
ifdef DMODE
  CPPFLAGS += -DDEBUG_MODE=$(DMODE)
else
  # Niv de trace minimum par default.
  CPPFLAGS += -DDEBUG_MODE=0
endif

# FLAG DEFENSIF
ifeq ($(DEFENSIF), yes)
  CPPFLAGS += -DDEFENSIF
endif

# --->
# ---> MAKE FLAGS
# --->

MAKE_FLAGS     =
ifeq ($(DEBUG),yes)
  MAKE_FLAGS     += DEBUG=yes
endif

ifeq ($(DEFENSIF), yes)
  MAKE_FLAGS     += DEFENSIF=yes
endif

ifdef DMODE
  MAKE_FLAGS     += DMODE=$(DMODE)
endif
