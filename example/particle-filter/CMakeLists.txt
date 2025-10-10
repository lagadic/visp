#############################################################################
#
# ViSP, open source Visual Servoing Platform software.
# Copyright (C) 2005 - 2024 by Inria. All rights reserved.
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
# See https://visp.inria.fr for more information.
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
# ViSP configuration file.
#
#############################################################################

cmake_minimum_required(VERSION 3.10)

project(example-kalman)

find_package(VISP REQUIRED visp_core visp_gui)

set(example_cpp)

list(APPEND example_cpp pf-nonlinear-example.cpp)
list(APPEND example_cpp pf-nonlinear-complex-example.cpp)

foreach(cpp ${example_cpp})
  visp_add_target(${cpp})
  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "examples")
  endif()
endforeach()

visp_add_test(pf-nonlinear-example-monothread pf-nonlinear-example -c ${SHORT_OPTION_TO_DISABLE_DISPLAY} --dt 3 --stdev-range 5 --stdev-elev-angle 0.5 --stdev-aircraft-vel 0.2>  --gt-X0 -500 --gt-Y0 1000 --gt-vX0 10 --gt-vY0 5 --max-distance-likelihood 50 -N 500 --seed 4224 --nb-threads 1 --ampli-max-X 20 --ampli-max-Y 200 --ampli-max-vX 1 --ampli-max-vY 0.5)
visp_add_test(pf-nonlinear-example-multithread pf-nonlinear-example -c ${SHORT_OPTION_TO_DISABLE_DISPLAY} --dt 3 --stdev-range 5 --stdev-elev-angle 0.5 --stdev-aircraft-vel 0.2>  --gt-X0 -500 --gt-Y0 1000 --gt-vX0 10 --gt-vY0 5 --max-distance-likelihood 50 -N 500 --seed 4224 --nb-threads -1 --ampli-max-X 20 --ampli-max-Y 200 --ampli-max-vX 1 --ampli-max-vY 0.5)
visp_add_test(pf-nonlinear-complex-example-monothread pf-nonlinear-complex-example -c ${SHORT_OPTION_TO_DISABLE_DISPLAY} --max-distance-likelihood 0.5 --ampli-max-X 0.25 --ampli-max-Y 0.25 --ampli-max-theta 0.1 -N 500 --nb-threads 1)
visp_add_test(pf-nonlinear-complex-example-multithread pf-nonlinear-complex-example -c ${SHORT_OPTION_TO_DISABLE_DISPLAY} --max-distance-likelihood 0.5 --ampli-max-X 0.25 --ampli-max-Y 0.25 --ampli-max-theta 0.1 -N 500 --nb-threads -1)
