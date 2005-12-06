#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.
#AC_CONFIG_MACRO_DIR(m4)

AC_PREREQ(2.57)
AC_INIT([ViSP],[2.1.0],[visp@irisa.fr])
AC_CONFIG_HEADER([include/visp/vpConfig.h])
dnl Pick up the needed m4 macros.
AM_ACLOCAL_INCLUDE(macros)

# Checks for programs.
AC_PROG_CXX
AC_PROG_RANLIB
AC_CHECK_TOOL(AR, ar crs, :)
AC_CHECK_PROG(HAVE_DOT, dot, YES, NO, ,)

AC_PROG_INSTALL
AC_PROG_MAKE_SET

########################################################################
# check for specific libraries
########################################################################

#-----------------------------------------------------------------------
# Check for -lxml2 library
# Warning: We suppose that include files are in /usr/include/libxml2
#AC_CHECK_LIB(xml2,main)
AC_CHECK_LIB(xml2,main,have_xml2=yes,have_xml2=no)
if test "x$have_xml2" = "xyes"; then
  LIBS="$LIBS -lxml2"
  CXXFLAGS="$CXXFLAGS -I/usr/include/libxml2"
fi 
#-----------------------------------------------------------------------
# Check for -lm library
AC_CHECK_LIB(m,main)
#-----------------------------------------------------------------------
# Check for -lgslcblas library
AC_CHECK_LIB(gslcblas,main)
#-----------------------------------------------------------------------
# Check for -lgsl library
AC_CHECK_LIB(gsl,main)
#-----------------------------------------------------------------------
# Check for -lcfox library 1394 acquisition on the mac

AC_CHECK_LIB(cfox,main,have_cfox=yes,have_cfox=no)
if test "x$have_cfox" = "xyes"; then
 AC_DEFINE([HAVE_LIBCFOX], 1,
            [Define to 1 if Cfox library is available])
  LIBS="$LIBS -lcfox -framework IOKit -framework Carbon "
fi 
#-----------------------------------------------------------------------
# Check for ieee 1394 capabilities
AC_CHECK_LIB(raw1394,main)
AC_CHECK_LIB(dc1394_control,main)
AC_CHECK_LIB(jpeg,main)
#-----------------------------------------------------------------------
# Check for X
AC_PATH_XTRA
#-----------------------------------------------------------------------
# Check for X11 library
AC_CHECK_LIB(X11, XOpenDisplay,,,$X_LIBS $X_EXTRA_LIBS)
#-----------------------------------------------------------------------
# Check for -lpng library
AC_CHECK_LIB(png, png_create_read_struct,,,$X_LIBS $X_EXTRA_LIBS)
#-----------------------------------------------------------------------
# Check for -lz library
AC_CHECK_LIB(z, png_create_read_struct,,,$X_LIBS $X_EXTRA_LIBS)
#-----------------------------------------------------------------------
# Check for -ljpeg library
AC_CHECK_LIB(jpeg, jpeg_std_error,,,$X_LIBS $X_EXTRA_LIBS)

########################################################################
# GTK
# AM_PATH_GTK(1.2.0,have_gtk=yes,have_gtk=no)
# if test "x$have_gtk" = "xyes" ; then 
#  echo "have gtk yes:	$have_gtk"
#  AC_DEFINE([HAVE_GTK],  1,
#            [Define to 1 if GTK library is available])
#  AC_SUBST(HAVE_GTK,yes)
#  LIBS="$LIBS $GTK_LIBS"
#  CXXFLAGS="$CXXFLAGS $GTK_CFLAGS"
#else
#echo "have gtk no:	$have_gtk"
#	fi 
########################################################################
# GTK
AM_PATH_GTK_2_0(2.0.0,have_gtk=yes,have_gtk=no)
if test "x$have_gtk" = "xyes" ; then 
 echo "have gtk yes:	$have_gtk"
  AC_DEFINE([HAVE_GTK],  1,
            [Define to 1 if GTK library is available])
  AC_SUBST(HAVE_GTK,yes)
  LIBS="$LIBS $GTK_LIBS"
  CXXFLAGS="$CXXFLAGS $GTK_CFLAGS"
else
 AM_PATH_GTK(1.2.0,have_gtk=yes,have_gtk=no)
 if test "x$have_gtk" = "xyes" ; then 
  echo "have gtk yes:	$have_gtk"
  AC_DEFINE([HAVE_GTK],  1,
            [Define to 1 if GTK library is available])
  AC_SUBST(HAVE_GTK,yes)
  LIBS="$LIBS $GTK_LIBS"
  CXXFLAGS="$CXXFLAGS $GTK_CFLAGS"
 else
  echo "have gtk no:	$have_gtk"
 fi 
fi 
#-----------------------------------------------------------------------
# Check for Qt library
BNV_HAVE_QT
if test "x$have_qt" = "xyes"; then
  AC_DEFINE([HAVE_QT], 1,
            [Define to 1 if Qt library is available])
  AC_SUBST(HAVE_QT,yes)
  CXXFLAGS="$CXXFLAGS $QT_CXXFLAGS"
  LIBS="$LIBS $QT_LIBS"
else
  LDFLAGS="$LDFLAGS $X_LIBS"
  AC_SUBST(HAVE_QT,no)
fi


#-----------------------------------------------------------------------
# Check for -lCoin library and if Qt exists for -lSoQt. else for -lSoXt
#SIM_AC_HAVE_COIN_IFELSE(have_coin=yes,have_coin=no)
#if test "x$have_coin" = "xyes"; then
#  AC_DEFINE([HAVE_COIN], 1,
#            [Define to 1 if Coin library is available])
#  AC_SUBST(HAVE_COIN,yes)
#  CPPFLAGS="$CPPFLAGS $sim_ac_coin_cppflags"
#  CXXFLAGS="$CXXFLAGS $sim_ac_coin_cxxflags"
#  LDFLAGS="$LDFLAGS $sim_ac_coin_ldflags"
#  LIBS="$LIBS $sim_ac_coin_libs"
#  if test "x$have_qt" = "xyes"; then
#    # Check for -lSoQt library
#    AC_LANG_PUSH(C++)       
#    AC_CHECK_LIB(SoQt,main)
#    AC_LANG_POP(C++)       
#  else
##    # Check for -lSoXt library
#    AC_LANG_PUSH(C++)       
#    AC_CHECK_LIB(SoXt,main)
#    AC_LANG_POP(C++)       
#  fi
#else
#  AC_SUBST(HAVE_COIN,no)
#fi
#-----------------------------------------------------------------------
# Check for -lCoin library and if Qt exists for -lSoQt. else for -lSoXt
SIM_AC_HAVE_COIN_IFELSE(have_coin=yes,have_coin=no)
if test "x$have_coin" = "yes"; then
  AC_DEFINE([HAVE_COIN], 1,
            [Define to 1 if Coin library is available])
  AC_SUBST(HAVE_COIN,yes)
  CPPFLAGS="$CPPFLAGS $sim_ac_coin_cppflags"
  CXXFLAGS="$CXXFLAGS $sim_ac_coin_cxxflags"
  LDFLAGS="$LDFLAGS $sim_ac_coin_ldflags"
  LIBS="$LIBS $sim_ac_coin_libs"
  if test "x$have_qt" = "xyes"; then
    # Check for -lSoQt library
    AC_LANG_PUSH(C++)       
    AC_CHECK_LIB(SoQt,main)
    AC_LANG_POP(C++)       
else
      # Check for -lSoXt library
  AC_LANG_PUSH(C++)       
  AC_CHECK_LIB(SoXt,main)
  AC_LANG_POP(C++)   
    fi
else
  AC_SUBST(HAVE_COIN,no)
fi

########################################################################
# Extra checks
########################################################################
#-----------------------------------------------------------------------
# Get the OS name
OS=`uname -s`
AC_SUBST(OS,$OS)
#-----------------------------------------------------------------------
# Get the hostname
AC_SUBST(ac_hostname,$ac_hostname)
#-----------------------------------------------------------------------
# Get the compiler version
CXX_VERSION=`$CXX -dumpversion`
AC_SUBST(CXX_VERSION,$CXX_VERSION)


#-----------------------------------------------------------------------
# Manage --doxygen-html-output option
AC_ARG_WITH([doxygen-output-directory],
  AC_HELP_STRING([--with-doxygen-output-directory=DIR],
		 [Location of generated doxygen documentation. By default in doc]),
                 DOXYGEN_OUTPUT_DIRECTORY=$withval, 
                 DOXYGEN_OUTPUT_DIRECTORY=.)
AC_SUBST(DOXYGEN_OUTPUT_DIRECTORY)

########################################################################
# check for compiler debug options
########################################################################
#-----------------------------------------------------------------------
# Remove -g option if it was fixed before automatically
CXXFLAGS=`echo $CXXFLAGS | sed -e "s;-g;;g"`

#-----------------------------------------------------------------------
# Manage the --enable-debug option
AC_MSG_CHECKING(whether to enable debugging)
AC_ARG_ENABLE(debug,
  AC_HELP_STRING([--enable-debug@<:@yes/no@:>@],
  [turn on debugging by adding -g compiler option [[default=no]] and add _g suffix to the build library names.]),enable_debug=yes, enable_debug=no)
if test "x$enable_debug" = "xyes"; then
  CXXFLAGS="$CXXFLAGS -g -Wno-deprecated"
  CPPFLAGS="$CPPFLAGS -DVP_DEBUG"
  SUFFIX="_g"
  AC_MSG_RESULT(yes)
else
  CXXFLAGS="$CXXFLAGS"
  SUFFIX=""
  AC_MSG_RESULT(no)
fi

#-----------------------------------------------------------------------
# Manage the --enable-insure option
AC_CHECK_PROG(have_insure, insure, yes, no, ,)
if test "x$have_insure" = "xyes"; then
  AC_DEFINE([HAVE_INSURE], 1,
            [Define to 1 if insure++ debugging tool is available])
  AC_SUBST(HAVE_INSURE,yes)
else
  AC_SUBST(HAVE_INSURE,no)
fi

if test "x$have_insure" = "xyes"; then
AC_MSG_CHECKING(whether to enable insure++ debugging tool)
AC_ARG_ENABLE(insure,
  AC_HELP_STRING([--enable-insure@<:@yes/no@:>@],
  [compile with insure++ debugging tool [[default=no]] and add _insure suffix to the build library names.]),enable_insure=yes, enable_insure=no)
if test "x$enable_insure" = "xyes"; then
  INSURE=insure
  CXXFLAGS="$CXXFLAGS -g -Wno-deprecated"
  CPPFLAGS="$CPPFLAGS -DVP_DEBUG"
  SUFFIX="_insure"
  AC_MSG_RESULT(yes)
else
#  INSURE=""
#  SUFFIX=""
  AC_MSG_RESULT(no)
fi
fi

AC_SUBST(SUFFIX,$SUFFIX)

########################################################################
# Fix the name of the librairies
########################################################################

VISP_LIBNAME=$PACKAGE_TARNAME-$PACKAGE_VERSION$SUFFIX
AC_SUBST(VISP_LIBNAME,$VISP_LIBNAME)


########################################################################
# Add compiler options
########################################################################
#-----------------------------------------------------------------------
# Add systematically compiler options: -Wall -Wunused
CXXFLAGS="$CXXFLAGS -Wall -Wunused -Werror"
#-----------------------------------------------------------------------
# Add systematically compiler options: -Wall -Wunused
CPPFLAGS="$CPPFLAGS -D__${OS}_"
#-----------------------------------------------------------------------
# Set VISP home path
relpath=`dirname $0`
VISP_HOME=`(cd $relpath; pwd)`
AC_SUBST(VISP_HOME)
#-----------------------------------------------------------------------
# Add systematically compiler options: -I$(VISP_DIR_INCLUDE)
VISP_DIR_INCLUDE="$VISP_HOME/include"
CXXFLAGS="$CXXFLAGS -I$VISP_DIR_INCLUDE -I."
#-----------------------------------------------------------------------
# Add systematically compiler options: -L$(VISP_LIB_PATH)
#VISP_LIB_PATH="$VISP_HOME/lib/$OS/$CXX_VERSION"
VISP_LIB_PATH="$VISP_HOME/lib"
LDFLAGS="$LDFLAGS -L$VISP_LIB_PATH"


########################################################################
# check for specific hardware
########################################################################

HAVE_FG_V4L
HAVE_FG_V4L2
IRISA_HAVE_FG_ICCOMP
IRISA_HAVE_BIT3_DRIVER
IRISA_HAVE_ROBOT_AFMA4
IRISA_HAVE_ROBOT_AFMA6
IRISA_HAVE_ROBOT_PTUEVI
IRISA_HAVE_ROBOT_VIC2500


########################################################################
# Add all the ViSP libraries
########################################################################
# sould be...
#LIBS="-l$LIBVISP $LIBS"
# Is for the moment...
LIBS="-l$VISP_LIBNAME $LIBS"

########################################################################
# Create Makefiles
########################################################################
AC_SUBST_FILE(build_mk)
build_mk=$VISP_HOME/configs/build.mk
AC_SUBST_FILE(debug_mk)
debug_mk=$VISP_HOME/configs/debug.mk
AC_SUBST_FILE(directory_mk)
directory_mk=$VISP_HOME/configs/directory.mk
AC_SUBST_FILE(lib_mk)
lib_mk=$VISP_HOME/configs/lib.mk
AC_SUBST_FILE(bin_mk)
bin_mk=$VISP_HOME/configs/bin.mk
AC_SUBST_FILE(clean_mk)
clean_mk=$VISP_HOME/configs/clean.mk
AC_SUBST_FILE(uninstall_mk)
uninstall_mk=$VISP_HOME/configs/uninstall.mk
AC_SUBST_FILE(depend_mk)
depend_mk=$VISP_HOME/configs/depend.mk

AC_CONFIG_FILES([
configs/build.mk
doc/config-doxygen
include/Makefile
include/visp/Makefile
Makefile
src/Makefile
src/camera/Makefile
src/debug/Makefile
src/exceptions/Makefile
src/math/Makefile
src/math/misc/Makefile
src/math/transformation/Makefile
src/math/matrix/Makefile
src/math/robust/Makefile
src/data-structure/Makefile
src/robot/Makefile
src/robot/robot/Makefile
src/robot/simulation-robot/Makefile
src/robot/real-robot/Makefile
src/robot/real-robot/afma6/Makefile
src/robot/real-robot/afma4/Makefile
src/servo/Makefile
src/simulator/Makefile
src/tools/Makefile
src/tools/io/Makefile
src/tools/geometry/Makefile
src/tools/time/Makefile
src/visual-feature/Makefile
src/image/Makefile
src/computer-vision/Makefile
src/computer-vision/pose-estimation/Makefile
src/computer-vision/homography-estimation/Makefile
src/video-device/Makefile
src/framegrabber-device/Makefile
src/framegrabber-device/IC-Comp/Makefile
src/framegrabber-device/1394/Makefile
src/framegrabber-device/OSX-cfox/Makefile
src/framegrabber-device/v4l2/Makefile
src/framegrabber-device/generic-framegrabber/Makefile
src/framegrabber-device/disk/Makefile
src/tracking/Makefile
src/tracking/dots/Makefile
src/tracking/moving-edges/Makefile
src/tracking/forward-projection/Makefile
src/tracking/feature-builder/Makefile
src/tracking/general-tracking-issues/Makefile
test/Makefile
test/pose/Makefile
test/servo-simu/Makefile
test/servo-afma4/Makefile
test/servo-afma6/Makefile
test/framegrabber/Makefile
test/feature/Makefile
test/math/Makefile
test/image/Makefile
test/tracking/Makefile
test/simulator/Makefile	
test/homography/Makefile	
example/Makefile
example/tracking/Makefile
example/tracking/vpDot/Makefile
example/image/Makefile
example/simulation/Makefile
example/pose-estimation/Makefile
])

########################################################################
# Manage insure ++
########################################################################
if test "x$enable_insure" = "xyes" || test "x$enable_debug" = "xyes"; then
  CXX="$INSURE $CXX"
  # Remove -O[123] option if it was fixed before automatically
  CXXFLAGS=`echo $CXXFLAGS | sed -e "s;-O1;;g"`
  CXXFLAGS=`echo $CXXFLAGS | sed -e "s;-O2;;g"`
  CXXFLAGS=`echo $CXXFLAGS | sed -e "s;-O3;;g"`
fi

AC_OUTPUT

########################################################################
# Display compiler options
########################################################################
echo ""
echo "Building options:"
echo "Hostname     : $ac_hostname"
echo "Debugging on : $enable_debug"
echo "CXX          : $CXX"
echo "CXX version  : $CXX_VERSION"
echo "SUFFIX       : $SUFFIX"
echo "CXXFLAGS     : $CXXFLAGS"
echo "CPPFLAGS     : $CPPFLAGS"
echo "LDFLAGS      : $LDFLAGS"
echo "LIBS         : $LIBS"
echo "GTK_LIBS	   : $GTK_LIBS"

