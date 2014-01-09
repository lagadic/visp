#!/bin/sh
#############################################################################
#
# $Id: distclean.sh,v 1.3 2006-05-29 09:58:22 fspindle Exp $
#
# This file is part of the ViSP software.
# Copyright (C) 2005 - 2014 by INRIA. All rights reserved.
# 
# This software is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# ("GPL") version 2 as published by the Free Software Foundation.
# See the file LICENSE.txt at the root directory of this source
# distribution for additional information about the GNU GPL.
#
# For using ViSP with software that can not be combined with the GNU
# GPL, please contact INRIA about acquiring a ViSP Professional 
# Edition License.
#
# See http://www.irisa.fr/lagadic/visp/visp.html for more information.
# 
# This software was developed at:
# INRIA Rennes - Bretagne Atlantique
# Campus Universitaire de Beaulieu
# 35042 Rennes Cedex
# France
# http://www.irisa.fr/lagadic
#
# If you have questions regarding the use of this file, please contact
# INRIA at visp@inria.fr
# 
# This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
# WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
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
$RM VISPBuildSettings.cmake
$RM VISPConfig.cmake
$RM VISPUse.cmake
$RM bin/visp-config

$RM -r Testing autom4te.cache

for i in `find . -type d -name "CMakeFiles" -print | sort`
    do
    	$RM -r $i;
	echo "$RM -r $i"
    done
for i in `find . -type d -name "CMakeTmp" -print | sort`
    do
    	$RM -r $i;
	echo "$RM -r $i"
    done
for i in `find . -type d -name "*.dir" -print | sort`
    do
    	$RM -r $i;
	echo "$RM -r $i"
    done
for i in `find . test -type d -name "debug" -print | sort`
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



for i in `find . -type f -name "*.rule" -print | sort`
    do
    	$RM  $i;
	echo "$RM $i"
    done
for i in `find . -type f -name "DartTestfile.txt" -print | sort`
    do
    	$RM $i;
	echo "$RM $i"
    done
for i in `find . -type f -name "*~" -print | sort`
    do
    	$RM $i;
	echo "$RM $i"
    done
for i in `find . -type f -name "*.plg" -print | sort`
    do
    	$RM $i;
	echo "$RM $i"
    done
for i in `find . -type f -name "*.vcproj*" -print | sort`
    do
    	$RM $i;
	echo "$RM $i"
    done
for i in `find . -type f -name "vc60.idb*" -print | sort`
    do
    	$RM $i;
	echo "$RM $i"
    done
for i in `find . -type f -name "*.dsp*" -print | sort`
    do
    	$RM $i;
	echo "$RM $i"
    done
for i in `find . -type f -name "*.dsw" -print | sort`
    do
    	$RM $i;
	echo "$RM $i"
    done
for i in `find . -type f -name "cmake_install.cmake" -print | sort`
    do
    	$RM $i;
	echo "$RM $i"
    done
for i in `find . -type f -name "*.opt" -print | sort`
    do
    	$RM $i;
	echo "$RM $i"
    done
for i in `find . -type f -name "*.sln" -print | sort`
    do
    	$RM $i;
	echo "$RM $i"
    done
for i in `find . -type f -name "*.ncb" -print | sort`
    do
    	$RM $i;
	echo "$RM $i"
    done
for i in `find . -type f -name "*.suo" -print | sort`
    do
    	$RM $i;
	echo "$RM $i"
    done
for i in `find . -type d \( -name Makefile \) -prune -o -type f -name "Makefile" -print | sort`
    do
    	$RM $i;
	echo "$RM $i"
    done
for i in `find . -type f -name "*.obj" -print | sort`
    do
    	$RM $i;
	echo "$RM $i"
    done
for i in `find . -type f -name "*.o" -print | sort`
    do
    	$RM $i;
	echo "$RM $i"
    done
for i in `find . -type f -name "*.bb" -print | sort`
    do
    	$RM $i;
	echo "$RM $i"
    done
for i in `find . -type f -name "*.bbg" -print | sort`
    do
    	$RM $i;
	echo "$RM $i"
    done
