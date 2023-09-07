import visp
from visp.core import ArrayDouble2D, RotationMatrix, Matrix

import numpy as np
import pytest

def test_np_array_modifies_vp_array():
  # Test that numpy is a view of array and that writing to numpy array modifies vpArray
  array = ArrayDouble2D(5, 5, 1.0)
  assert array.getRows() == array.getCols() == 5
  array_np = np.array(array, copy=False)
  assert array_np.shape == (5, 5)
  assert np.all(array_np == 1.0)
  array_np[0:2, 0:2] = 2
  assert array.getMinValue() == 1 and array.getMaxValue() == 2



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

def test_rotation_representations_not_writable():
  # Test that some class have non writable numpy arrays
  R = RotationMatrix()
  R_np = np.array(R, copy=False)
  with pytest.raises(ValueError):
    R_np[0, 0] = 1