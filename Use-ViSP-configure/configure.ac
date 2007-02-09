#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.
#AC_CONFIG_MACRO_DIR(m4)

AC_PREREQ(2.57)
AC_INIT([example],[1.0.0],[Fabien.Spindler@irisa.fr])
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
  CPPFLAGS="$CPPFLAGS -DDEBUG"
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
  CPPFLAGS="$CPPFLAGS -DDEBUG"
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

PROJECT_LIBNAME=$PACKAGE_TARNAME-$PACKAGE_VERSION$SUFFIX
AC_SUBST(PROJECT_LIBNAME,$PROJECT_LIBNAME)


########################################################################
# Add compiler options
########################################################################
#-----------------------------------------------------------------------
# Add systematically compiler options: -Wall -Wunused
CXXFLAGS="$CXXFLAGS -Wall -Wunused -Werror"
#-----------------------------------------------------------------------
# Set PROJECT home path
relpath=`dirname $0`
PROJECT_HOME=`(cd $relpath; pwd)`
AC_SUBST(PROJECT_HOME)
#-----------------------------------------------------------------------
# Add systematically compiler options: -I$(PROJECT_DIR_INCLUDE)
PROJECT_DIR_INCLUDE="$PROJECT_HOME/include"
CXXFLAGS="$CXXFLAGS -I$PROJECT_DIR_INCLUDE -I."
#-----------------------------------------------------------------------
# Add systematically compiler options: -L$(PROJECT_LIB_PATH)
#PROJECT_LIB_PATH="$PROJECT_HOME/lib/$OS/$CXX_VERSION"
PROJECT_LIB_PATH="$PROJECT_HOME/lib"
LDFLAGS="$LDFLAGS -L$PROJECT_LIB_PATH"


#######################################################################
# check for ViSP library
########################################################################
AC_HAVE_VISP_IFELSE(have_visp=yes,have_visp=no)
if test "x$have_visp" = "xyes"; then
  CXXFLAGS="$CXXFLAGS $ac_visp_cflags "
  LIBS="$LIBS $ac_visp_libs "
fi

########################################################################
# Add the PROJECT libraries
########################################################################
LIBS="-l$PROJECT_LIBNAME $LIBS"

########################################################################
# Create Makefiles
########################################################################
AC_SUBST_FILE(build_mk)
build_mk=$PROJECT_HOME/configs/build.mk
AC_SUBST_FILE(debug_mk)
debug_mk=$PROJECT_HOME/configs/debug.mk
AC_SUBST_FILE(directory_mk)
directory_mk=$PROJECT_HOME/configs/directory.mk
AC_SUBST_FILE(lib_mk)
lib_mk=$PROJECT_HOME/configs/lib.mk
AC_SUBST_FILE(bin_mk)
bin_mk=$PROJECT_HOME/configs/bin.mk
AC_SUBST_FILE(clean_mk)
clean_mk=$PROJECT_HOME/configs/clean.mk
AC_SUBST_FILE(uninstall_mk)
uninstall_mk=$PROJECT_HOME/configs/uninstall.mk
AC_SUBST_FILE(depend_mk)
depend_mk=$PROJECT_HOME/configs/depend.mk

AC_CONFIG_FILES([
configs/build.mk
doc/config-doxygen
Makefile
include/Makefile
include/example/Makefile
src/Makefile
src/example/Makefile
example/Makefile
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
