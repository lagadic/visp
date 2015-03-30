# Configuration checks
include(CheckIncludeFile)
include(CheckIncludeFiles)
include(CheckCXXSourceCompiles)

macro(check_math_expr _expr _var)
    check_cxx_source_compiles("
#include <cmath>
int main(int argc, char ** argv)
{
    (void)${_expr};
    return 0;
}
" ${_var})
endmacro()

check_include_files("float.h"       HAVE_FLOAT_H)
check_math_expr("isnan(1.0)"        HAVE_FUNC_ISNAN)
check_math_expr("std::isnan(1.0)"   HAVE_FUNC_STD_ISNAN)

if(HAVE_FLOAT_H)
    # The version that should work with MSVC
    check_cxx_source_compiles("
#include <float.h>
int main(int argc, char ** argv)
{
    (void)_isnan(1.0);
    return 0;
}
" HAVE_FUNC__ISNAN)
else()
    set(HAVE_FUNC__ISNAN FALSE)
endif()
