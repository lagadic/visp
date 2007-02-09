                            Use-ViSP-configure
			      Release  1.0.0

         Copyright Projet Lagadic / IRISA-INRIA Rennes, 2007
                  www: http://www.irisa.fr/lagadic



This project is using the configure build system and shows how to use ViSP
(http://www.irisa.fr/lagadic/visp) as a third party library.

automake -a
aclocal -I macros
autoconf

./configure --with-visp-install-bin=/usr/local/bin
   Specify that the visp-config shell script is located in /usr/local/bin
   By default, without --with-visp-install-bin option, the search
   directory is /usr/bin

make
make html-doc

--
Fabien Spindler


