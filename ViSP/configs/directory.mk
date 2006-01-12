%_MKDIR:
	@if [ ! -d $* ] ;\
        then \
                echo "*" ; \
                echo "* Creation of the $* directory..." ; \
                echo "*" ; \
                mkdir -p $* ; \
        fi ;

BASE_DIRS = \
	$(VISP_OBJ_PATH)_MKDIR \
	$(VISP_DEP_PATH)_MKDIR \
	$(VISP_LIB_PATH)_MKDIR \
	$(VISP_BIN_PATH)_MKDIR 

directory: $(BASE_DIRS)

