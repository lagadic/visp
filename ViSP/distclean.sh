#!/bin/sh
#############################################################################
#
# $Id: distclean.sh,v 1.3 2006-05-29 09:58:22 fspindle Exp $
#
# Copyright (C) 1998-2010 Inria. All rights reserved.
#
# This software was developed at:
# IRISA/INRIA Rennes
# Projet Lagadic
# Campus Universitaire de Beaulieu
# 35042 Rennes Cedex
# http://www.irisa.fr/lagadic
#
# This file is part of the ViSP toolkit
#
# This file may be distributed under the terms of the Q Public License
# as defined by Trolltech AS of Norway and appearing in the file
# LICENSE included in the packaging of this file.
#
# Licensees holding valid ViSP Professional Edition licenses may
# use this file in accordance with the ViSP Commercial License
# Agreement provided with the Software.
#
# This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
# WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
#
# Contact visp@irisa.fr if any conditions of this licensing are
# not clear to you.
#
# Description:
# Cleaning cmake produced files.
#
# Authors:
# Fabien Spindler
#
#############################################################################


RM="rm -f"

$RM CMakeCache.txt CMakeOutput.log config.log warning.log cmake.check_cache
$RM install_manifest.txt DartConfiguration.tcl
$RM doc/config-doxygen warning.log cmake_uninstall.cmake
$RM include/visp/vpConfig.h
$RM lib/VISPBuildSettings.cmake
$RM lib/VISPConfig.cmake
$RM lib/VISPLibraryDepends.cmake
$RM lib/VISPUse.cmake
$RM bin/visp-config

$RM -r Testing autom4te.cache

for i in `find -type d -name "CMakeFiles" -print | sort`
    do
    	$RM -r $i;
	echo "$RM -r $i"
    done
for i in `find -type d -name "CMakeTmp" -print | sort`
    do
    	$RM -r $i;
	echo "$RM -r $i"
    done
for i in `find -type d -name "*.dir" -print | sort`
    do
    	$RM -r $i;
	echo "$RM -r $i"
    done
for i in `find test -type d -name "debug" -print | sort`
    do
    	$RM -r $i;
	echo "$RM -r $i"
    done
for i in `find lib -type d -name "debug" -print | sort`
    do
    	$RM -r $i;
	echo "$RM -r $i"
    done
for i in `find example -type d -name "debug" -print | sort`
    do
    	$RM -r $i;
	echo "$RM -r $i"
    done



for i in `find -type f -name "*.rule" -print | sort`
    do
    	$RM  $i;
	echo "$RM $i"
    done
for i in `find -type f -name "DartTestfile.txt" -print | sort`
    do
    	$RM $i;
	echo "$RM $i"
    done
for i in `find -type f -name "*~" -print | sort`
    do
    	$RM $i;
	echo "$RM $i"
    done
for i in `find -type f -name "*.plg" -print | sort`
    do
    	$RM $i;
	echo "$RM $i"
    done
for i in `find -type f -name "*.vcproj*" -print | sort`
    do
    	$RM $i;
	echo "$RM $i"
    done
for i in `find -type f -name "vc60.idb*" -print | sort`
    do
    	$RM $i;
	echo "$RM $i"
    done
for i in `find -type f -name "*.dsp*" -print | sort`
    do
    	$RM $i;
	echo "$RM $i"
    done
for i in `find -type f -name "*.dsw" -print | sort`
    do
    	$RM $i;
	echo "$RM $i"
    done
for i in `find -type f -name "cmake_install.cmake" -print | sort`
    do
    	$RM $i;
	echo "$RM $i"
    done
for i in `find -type f -name "*.opt" -print | sort`
    do
    	$RM $i;
	echo "$RM $i"
    done
for i in `find -type f -name "*.sln" -print | sort`
    do
    	$RM $i;
	echo "$RM $i"
    done
for i in `find -type f -name "*.ncb" -print | sort`
    do
    	$RM $i;
	echo "$RM $i"
    done
for i in `find -type f -name "*.suo" -print | sort`
    do
    	$RM $i;
	echo "$RM $i"
    done
for i in `find -type f -name "Makefile" -print | sort`
    do
    	$RM $i;
	echo "$RM $i"
    done
for i in `find -type f -name "*.obj" -print | sort`
    do
    	$RM $i;
	echo "$RM $i"
    done
for i in `find -type f -name "*.o" -print | sort`
    do
    	$RM $i;
	echo "$RM $i"
    done
for i in `find -type f -name "*.bb" -print | sort`
    do
    	$RM $i;
	echo "$RM $i"
    done
for i in `find -type f -name "*.bbg" -print | sort`
    do
    	$RM $i;
	echo "$RM $i"
    done
