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
