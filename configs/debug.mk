# --->
# ---> CFLAGS
# --->

# ceci est positionné directement par configure
#ifeq ($(DEBUG), yes)
#  CXXFLAGS += -g -DDEBUG -Wno-deprecated
#else
#  CXXFLAGS += -O3 -Werror
#endif

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
