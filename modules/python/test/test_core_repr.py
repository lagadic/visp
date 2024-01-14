#############################################################################
#
# ViSP, open source Visual Servoing Platform software.
# Copyright (C) 2005 - 2023 by Inria. All rights reserved.
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
# ViSP Python bindings test
#
#############################################################################

import visp
from visp.core import ArrayDouble2D, RotationMatrix, Matrix, HomogeneousMatrix, PoseVector

import numpy as np
import pytest

def test_array_operations():
  array1 = ArrayDouble2D(2, 2, 1)
  array2 = ArrayDouble2D(2, 2, 1)
  assert array1 == array2

def test_matrix_operations():
  m1 = Matrix(4, 4, 2.0)
  m2 = Matrix(4, 4, 1.0)
  m3 = Matrix(4, 4, 3.0)
  m4 = Matrix(4, 4, 6 * 4)

  assert m1 + m2 == m3
  assert m3 - m1 == m2
  assert m1 * m3 == m4
  assert m2 * 2 == m1

def test_rotation_repr_can_be_defined_by_hand():
  R = RotationMatrix()
