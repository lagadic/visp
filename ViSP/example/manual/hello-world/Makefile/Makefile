CXX            = g++
VISP_BUILD_DIR = /local/soft/ViSP/ViSP-build
VISP_CFLAGS    = `$(VISP_BUILD_DIR)/bin/visp-config --cflags`
VISP_LDFLAGS   = `$(VISP_BUILD_DIR)/bin/visp-config --libs`

HelloWorld: HelloWorld.cpp
	$(CXX) $(VISP_CFLAGS) -o HelloWorld HelloWorld.cpp $(VISP_LDFLAGS)

clean:
	rm -f *~ HelloWorld
