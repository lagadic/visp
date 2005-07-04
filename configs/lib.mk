OBJECTS_TMP =$(SOURCES:%.cpp=$(VISP_OBJ_PATH)/%.o)
OBJECTS     =$(OBJECTS_TMP:%.c=$(VISP_OBJ_PATH)/%.o)

DEPFILES_TMP=$(SOURCES:%.cpp=$(VISP_DEP_PATH)/%.P)
DEPFILES    =$(DEPFILES_TMP:%.c=$(VISP_DEP_PATH)/%.P)

# Be aware: with insure option -MD puts the dependency file in the same
# directory than the source file.  With classic compilators, -MD puts the
# dependency file in the target directory

ifeq ($(SUFFIX), _insure)
DEP_FILE	= $*.d
else
DEP_FILE	= $(VISP_OBJ_PATH)/$*.d
endif

$(LIB) : $(OBJECTS)
	$(AR) $@ $(VISP_OBJ_PATH)/*.o

# Rule to generate moc files for Qt
m%.cpp: %.h
	@echo "*"
	@echo "* Construct MOC file $@ "
	@echo "*"
	$(MOC) -o $@ $<

# Rule for combining compilation and dependency generation:
# - put objects in $(VISP_OBJ_PATH)
# - put dependencies in $(VISP_DEP_PATH)

$(VISP_OBJ_PATH)/%.o : %.cpp
	@echo "*"
	@echo "* Build $@ from $<  "
	@echo "*"
	$(CXXALL) -MD -o $@ -c $<
	@cp $(DEP_FILE) $(VISP_DEP_PATH)/$*.P; \
	sed -e 's/#.*//' -e 's/^[^:]*: *//' -e 's/ *\\$$//' \
	    -e '/^$$/ d' -e 's/$$/ :/' < $(DEP_FILE)\
	    >> $(VISP_DEP_PATH)/$*.P; \
	rm -f $(DEP_FILE)


$(VISP_OBJ_PATH)/%.o : %.c
	@echo "*"
	@echo "* Build $@ from $<  "
	@echo "*"
	$(CXXALL) -MD -o $@ -c $<
	@cp $(DEP_FILE) $(VISP_DEP_PATH)/$*.P; \
	sed -e 's/#.*//' -e 's/^[^:]*: *//' -e 's/ *\\$$//' \
	    -e '/^$$/ d' -e 's/$$/ :/' < $(DEP_FILE)\
	    >> $(VISP_DEP_PATH)/$*.P; \
	rm -f $(DEP_FILE)

