#############################################################################
#
# ViSP, open source Visual Servoing Platform software.
# Copyright (C) 2005 - 2019 by Inria. All rights reserved.
#
# This software is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
# See the file LICENSE.txt at the root directory of this source
# distribution for additional information about the GNU GPL.
#
# For using ViSP with software that can not be combined with the GNU
# GPL, please contact Inria about acquiring a ViSP Professional
# Edition License.
#
# See http://visp.inria.fr for more information.
#
# This software was developed at:
# Inria Rennes - Bretagne Atlantique
# Campus Universitaire de Beaulieu
# 35042 Rennes Cedex
# France
#
# If you have questions regarding the use of this file, please contact
# Inria at visp@inria.fr
#
# This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
# WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
#
# Description:
# Try to find isnan macro, std::isnan function or _isnan function.
#
# Authors:
# Souriya Trinh
#
#############################################################################

include(CheckIncludeFile)
include(CheckIncludeFiles)
include(CheckCXXSourceCompiles)

macro(check_math_expr _expr _var)
    unset(${_var} CACHE)
    # Since check_cxx_source_compiles() doesn't consider CXX_STANDARD we add the corresponding flag manually
    if(USE_CXX11 AND CXX11_CXX_FLAGS)
      set(CMAKE_REQUIRED_FLAGS ${CXX11_CXX_FLAGS})
    endif()
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
    unset(HAVE_FUNC__ISNAN CACHE)
    # Since check_cxx_source_compiles() doesn't consider CXX_STANDARD we add the corresponding flag manually
    if(USE_CXX11 AND CXX11_CXX_FLAGS)
      set(CMAKE_REQUIRED_FLAGS ${CXX11_CXX_FLAGS})
    endif()
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

