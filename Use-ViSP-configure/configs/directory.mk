directory:
	@if [ ! -d $(PROJECT_DIR_OBJ) ] ;\
	then \
		echo "*" ; \
		echo "* Creation of the $(PROJECT_DIR_OBJ) directory..." ; \
		echo "*" ; \
		mkdir $(PROJECT_DIR_OBJ) ; \
	fi ;
	@if [ ! -d $(PROJECT_OBJ_PATH) ] ;\
	then \
		echo "*" ; \
		echo "* Creation of the $(PROJECT_OBJ_PATH) directory..." ; \
		echo "*" ; \
		mkdir $(PROJECT_OBJ_PATH) ; \
	fi ;
	@if [ ! -d $(PROJECT_DIR_DEP) ] ;\
	then \
		echo "*" ; \
		echo "* Creation of the $(PROJECT_DIR_DEP) directory..." ; \
		echo "*" ; \
		mkdir $(PROJECT_DIR_DEP) ; \
	fi ;
	@if [ ! -d $(PROJECT_DEP_PATH) ] ;\
	then \
		echo "*" ; \
		echo "* Creation of the $(PROJECT_DEP_PATH) directory..." ; \
		echo "*" ; \
		mkdir $(PROJECT_DEP_PATH) ; \
	fi ;
	@if [ ! -d $(PROJECT_LIB_PATH) ] ;\
	then \
		echo "*" ; \
		echo "* Creation of the $(PROJECT_LIB_PATH) directory..." ; \
		echo "*" ; \
		mkdir $(PROJECT_LIB_PATH) ; \
	fi ;
	@if [ ! -d $(PROJECT_BIN_PATH) ] ;\
	then \
		echo "*" ; \
		echo "* Creation of the $(PROJECT_BIN_PATH) directory..." ; \
		echo "*" ; \
		mkdir $(PROJECT_BIN_PATH) ; \
	fi ;
