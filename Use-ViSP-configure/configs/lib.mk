OBJECTS_TMP =$(SOURCES:%.cpp=$(PROJECT_OBJ_PATH)/%.o)
OBJECTS     =$(OBJECTS_TMP:%.c=$(PROJECT_OBJ_PATH)/%.o)

DEPFILES_TMP=$(SOURCES:%.cpp=$(PROJECT_DEP_PATH)/%.P)
DEPFILES    =$(DEPFILES_TMP:%.c=$(PROJECT_DEP_PATH)/%.P)

# Be aware: with insure option -MD puts the dependency file in the same
# directory than the source file.  With classic compilators, -MD puts the
# dependency file in the target directory

ifeq ($(SUFFIX), _insure)
DEP_FILE	= $*.d
else
DEP_FILE	= $(PROJECT_OBJ_PATH)/$*.d
endif

$(LIB) : $(OBJECTS)
	$(AR) $@ $(PROJECT_OBJ_PATH)/*.o

# Rule to generate moc files for Qt
m%.cpp: %.h
	@echo "*"
	@echo "* Construct MOC file $@ "
	@echo "*"
	$(MOC) -o $@ $<

# Rule for combining compilation and dependency generation:
# - put objects in $(PROJECT_OBJ_PATH)
# - put dependencies in $(PROJECT_DEP_PATH)
# - add Makefile in the list of dependencies
$(PROJECT_OBJ_PATH)/%.o : %.cpp
	@echo "*"
	@echo "* Build $@ from $<  "
	@echo "*"
	$(CXXALL) -MD -o $@ -c $<
	@sed -e 's/\($*.o\)*[:]/\1: Makefile/' < $(DEP_FILE) \
	    > $(PROJECT_DEP_PATH)/$*.P; \
	sed -e 's/#.*//' -e 's/^[^:]*: *//' -e 's/ *\\$$//' \
	    -e '/^$$/ d' -e 's/$$/ :/' < $(DEP_FILE) \
	    >> $(PROJECT_DEP_PATH)/$*.P; \
	rm -f $(DEP_FILE)

#	@cp $(DEP_FILE) $(PROJECT_DEP_PATH)/$*.P; \
#	sed -e 's/#.*//' -e 's/^[^:]*: *//' -e 's/ *\\$$//' \
#	    -e '/^$$/ d' -e 's/$$/ :/' < $(DEP_FILE)\
#	    >> $(PROJECT_DEP_PATH)/$*.P; \
#	rm -f $(DEP_FILE)


$(PROJECT_OBJ_PATH)/%.o : %.c
	@echo "*"
	@echo "* Build $@ from $<  "
	@echo "*"
	$(CXXALL) -MD -o $@ -c $<
	@sed -e 's/\($*.o\)*[:]/\1: Makefile/' < $(DEP_FILE) \
	    > $(PROJECT_DEP_PATH)/$*.P; \
	sed -e 's/#.*//' -e 's/^[^:]*: *//' -e 's/ *\\$$//' \
	    -e '/^$$/ d' -e 's/$$/ :/' < $(DEP_FILE) \
	    >> $(PROJECT_DEP_PATH)/$*.P; \
	rm -f $(DEP_FILE)

#	@cp $(DEP_FILE) $(PROJECT_DEP_PATH)/$*.P; \
#	sed -e 's/#.*//' -e 's/^[^:]*: *//' -e 's/ *\\$$//' \
#	    -e '/^$$/ d' -e 's/$$/ :/' < $(DEP_FILE)\
#	    >> $(PROJECT_DEP_PATH)/$*.P; \
#	rm -f $(DEP_FILE)

