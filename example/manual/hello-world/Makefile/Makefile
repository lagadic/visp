CXX            = g++

# In a Makefile there are two ways to build a project with ViSP as third party:
# 1. Using visp-config script provided either in <visp build tree>/bin/visp-config
#    or in <visp install tree>/bin/visp-config (typically in /usr/bin/visp-config)
#VISP_BUILD_DIR = /local/soft/ViSP/ViSP-build
#VISP_CFLAGS    = `$(VISP_BUILD_DIR)/bin/visp-config --cflags`
#VISP_LDFLAGS   = `$(VISP_BUILD_DIR)/bin/visp-config --libs`

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
	rm -f *~ HelloWorld
