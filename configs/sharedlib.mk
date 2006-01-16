shared-lib: $(SHARED_LIB_PATH)

$(SHARED_LIB_PATH): $(VISP_OBJ_PATH)/*.o
	$(CXXALL) -shared -Wl,-soname,$(SHARED_LIB_NAME) $^ -o $@ $(LDFLAGS) $(SHARED_LIBS)

lib-link:
	rm -f $(VISP_LIB_PATH)/libvisp-2$(SUFFIX).a
	ln -s $(STATIC_LIB_PATH) $(VISP_LIB_PATH)/libvisp-2$(SUFFIX).a
	rm -f $(VISP_LIB_PATH)/libvisp-2$(SUFFIX).so
	ln -s $(SHARED_LIB_PATH) $(VISP_LIB_PATH)/libvisp-2$(SUFFIX).so
