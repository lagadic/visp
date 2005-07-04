DEPFILES_TMP=$(SOURCES:%.cpp=$(VISP_DEP_PATH)/%.P)
DEPFILES    =$(DEPFILES_TMP:%.c=$(VISP_DEP_PATH)/%.P)

ifeq ($(SUFFIX), _insure)
DEP_FILE	= $*.d
else
DEP_FILE	= $(*F)$(SUFFIX).d
endif

# Rule for combining compilation and dependency generation
# - put binaries in current directory
# - put dependencies in $(VISP_DEP_PATH)#$(VISP_LIBNAME)
%$(SUFFIX).exe : %.cpp
	@echo "*"
	@echo "* Create the binary file $@ for $< "
	@echo "*"
	$(CXXALL) -MD -o $@ $< $(LDFLAGS) $(LIBS)
	@cp $(DEP_FILE) $(VISP_DEP_PATH)/$(*F)$(SUFFIX).P; \
	sed -e 's/#.*//' -e 's/^[^:]*: *//' -e 's/ *\\$$//' \
	    -e '/^$$/ d' -e 's/$$/ :/' < $(DEP_FILE) \
	    >> $(VISP_DEP_PATH)/$(*F).P; \
	rm -f $(DEP_FILE)
