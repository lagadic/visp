directory:
	@if [ ! -d $(VISP_DIR_OBJ) ] ;\
	then \
		echo "*" ; \
		echo "* Creation of the $(VISP_DIR_OBJ) directory..." ; \
		echo "*" ; \
		mkdir $(VISP_DIR_OBJ) ; \
	fi ;
	@if [ ! -d $(VISP_OBJ_PATH) ] ;\
	then \
		echo "*" ; \
		echo "* Creation of the $(VISP_OBJ_PATH) directory..." ; \
		echo "*" ; \
		mkdir $(VISP_OBJ_PATH) ; \
	fi ;
	@if [ ! -d $(VISP_DIR_DEP) ] ;\
	then \
		echo "*" ; \
		echo "* Creation of the $(VISP_DIR_DEP) directory..." ; \
		echo "*" ; \
		mkdir $(VISP_DIR_DEP) ; \
	fi ;
	@if [ ! -d $(VISP_DEP_PATH) ] ;\
	then \
		echo "*" ; \
		echo "* Creation of the $(VISP_DEP_PATH) directory..." ; \
		echo "*" ; \
		mkdir $(VISP_DEP_PATH) ; \
	fi ;
#	@if [ ! -d $(VISP_DIR_LIB)/$(OS) ] ;\
#	then \
#		echo "*" ; \
#		echo "* Creation of the $(VISP_DIR_LIB)/$(OS) directory..." ; \
#		echo "*" ; \
#		mkdir $(VISP_DIR_LIB)/$(OS) ; \
#	fi ;
#	@if [ ! -d $(VISP_LIB_PATH) ] ;\
#	then \
#		echo "*" ; \
#		echo "* Creation of the $(VISP_LIB_PATH) directory..." ; \
#		echo "*" ; \
#		mkdir $(VISP_LIB_PATH) ; \
#	fi ;
