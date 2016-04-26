#makefile for ftconvert and calinfo
#use 'make ftconvert' to make ftconvert sample and 'make calinfo' to make calinfo
#uses gcc, needs modifications for other compilers.
object_compile = gcc -c
exe_compile = gcc -o
remove_command = rm
ftconvert: ftconvert.o ftconfig.o ftrt.o dom.o expatls.o node.o stack.o xmlparse.o xmlrole.o xmltok.o
	$(exe_compile) ftconvert.exe ftconvert.o ftconfig.o ftrt.o dom.o expatls.o node.o stack.o xmlparse.o xmlrole.o xmltok.o
ftconvert.o: ftconvert.c ../atidaq/ftconfig.h 
	$(object_compile) ftconvert.c
ftconfig.o: ../atidaq/ftconfig.c ../atidaq/ftconfig.h
	$(object_compile) ../atidaq/ftconfig.c
ftrt.o: ../atidaq/ftrt.c ../atidaq/ftrt.h  ../atidaq/ftsharedrt.h
	$(object_compile) ../atidaq/ftrt.c
dom.o: ../atidaq/dom.c
	$(object_compile) ../atidaq/dom.c
expatls.o: ../atidaq/expatls.c
	$(object_compile) ../atidaq/expatls.c
node.o: ../atidaq/node.c
	$(object_compile) ../atidaq/node.c
stack.o: ../atidaq/stack.c
	$(object_compile) ../atidaq/stack.c
xmlparse.o: ../atidaq/xmlparse.c
	$(object_compile) ../atidaq/xmlparse.c
xmlrole.o: ../atidaq/xmlrole.c
	$(object_compile) ../atidaq/xmlrole.c
xmltok.o: ../atidaq/xmltok.c
	$(object_compile) ../atidaq/xmltok.c
calinfo: calinfo.o ftconfig.o ftrt.o dom.o expatls.o node.o stack.o xmlparse.o xmlrole.o xmltok.o
	$(exe_compile) calinfo.exe calinfo.o ftconfig.o ftrt.o dom.o expatls.o node.o stack.o xmlparse.o xmlrole.o xmltok.o
calinfo.o: calinfo.c ../atidaq/ftconfig.h
	$(object_compile) calinfo.c
clean:
	$(remove_command) *.o *.exe
