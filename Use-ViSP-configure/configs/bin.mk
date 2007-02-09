DEPFILES_TMP=$(SOURCES:%.cpp=$(PROJECT_DEP_PATH)/%.P)
DEPFILES    =$(DEPFILES_TMP:%.c=$(PROJECT_DEP_PATH)/%.P)

ifeq ($(SUFFIX), _insure)
DEP_FILE	= $*.d
else
DEP_FILE	= $(*F)$(SUFFIX).d
endif

# Rule for combining compilation and dependency generation
# - put binaries in $(PROJECT_BIN_PATH) directory
# - put dependencies in $(PROJECT_DEP_PATH)#$(PROJECT_LIBNAME)
# - add Makefile in the list of dependencies
# - add ViSP library to the list of dependencies
%$(SUFFIX).exe : %.cpp $(PROJECT_LIB_PATH)/lib$(PROJECT_LIBNAME).a
	@echo "*"
	@echo "* Create the binary file $@ for $< "
	@echo "*"
	$(CXXALL) -MD -o $@ $< $(LDFLAGS) $(LIBS)
	@sed -e 's/\($*.o\)*[:]/\1: Makefile/' \
	    < $(DEP_FILE) \
	    > $(PROJECT_DEP_PATH)/$(*F)$(SUFFIX).P; \
	sed -e 's/#.*//' -e 's/^[^:]*: *//' -e 's/ *\\$$//' \
	    -e '/^$$/ d' -e 's/$$/ :/' < $(DEP_FILE) \
	    >> $(PROJECT_DEP_PATH)/$(*F).P; \
	rm -f $(DEP_FILE); \
	cd $(PROJECT_BIN_PATH); \
	rm -f $@; \
	ln -sf `find $(PROJECT_HOME) -name $@|grep -v CVS` $@
