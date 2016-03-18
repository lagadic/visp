CXX            = g++

# In a Makefile there are two ways to build a project with ViSP as third party:
# 1. Using visp-config script provided either in <visp build tree>/bin/visp-config
#    or in <visp install tree>/bin/visp-config (typically in /usr/bin/visp-config)
#VISP_BUILD_DIR = /local/soft/ViSP/ViSP-build
#VISP_CFLAGS    = `$(VISP_BUILD_DIR)/bin/visp-config --cflags`
#VISP_LDFLAGS   = `$(VISP_BUILD_DIR)/bin/visp-config --libs`

#   Warning on OS X :
#   ———————————————————————————————
#   When visp-config is used from <visp build tree>/bin/visp-config
#   you may get the following error on OS X:
#   $ ./HelloWorld
#   dyld: Library not loaded: @rpath/libvisp_core.3.0.dylib
#     Referenced from: /Users/fspindle/soft/ViSP/ViSP-dev/visp/example/manual/hello-world/Makefile/./HelloWorld
#     Reason: image not found
#   Trace/BPT trap: 5
#
#   To indicate the location of the libraries on OS X, set DYLD_FALLBACK_LIBRARY_PATH like
#   $ export DYLD_FALLBACK_LIBRARY_PATH=<visp build tree>/lib

#   Warning on other linux platforms (Ubuntu for example) :
#   ———————————————————————————————
#   When visp-config is used from <visp build tree>/bin/visp-config
#   you may get the following error on Ubuntu:
#   $ ./HelloWorld
#   $ ./HelloWorld: error while loading shared libraries: libvisp_core.so.3.0: cannot open
#     shared object file: No such file or directory
#
#   To indicate the location of the libraries on linux, set LD_LIBRARY_PATH like
#   $ export LD_LIBRARY_PATH=<visp build tree>/lib


# 2. Using pkg-config on visp.pc file provided in <visp install tree>/lib/x86_64-linux-gnu/pkgconfig/visp.pc 
#    (typically in /usr/lib/x86_64-linux-gnu/pkgconfig/visp.pc)
#    If visp.pc pkg-config file is not found, you may set PKG_CONFIG_PATH 
#    environment variable with the path to access to visp.pc
#    For example: export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:<visp install tree>/lib/x86_64-linux-gnu/pkgconfig
VISP_CFLAGS    = `pkg-config --cflags visp`
VISP_LDFLAGS   = `pkg-config --libs visp`


HelloWorld: HelloWorld.cpp
	$(CXX) $(VISP_CFLAGS) -o HelloWorld HelloWorld.cpp $(VISP_LDFLAGS)

clean:
	rm -f *~ HelloWorld myimage.pgm
