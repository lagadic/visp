SHELL=/bin/sh

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
for i in `find -type f -name "*.{bb,bbg}" -print | sort`
    do
    	$RM $i;
	echo "$RM $i"
    done
