clean:
	rm -f *%
	rm -f *~
	rm -f *.o
	@for i in `find -type f -name tca.* -print | sort` ; do\
		rm -f $$i;\
		echo "rm -f " $$i;\
	done
	@for i in `find -type d -name pchdir -print | sort` ; do\
		rm -fr $$i;\
		echo "rm -fr " $$i;\
	done
	@for i in `find -type f -name *.d -print | sort` ; do\
		rm -f $$i;\
		echo "rm -f " $$i;\
	done
	@for i in `find -type f -name .inslog2 -print | sort` ; do\
		rm -f $$i;\
		echo "rm -f " $$i;\
	done
