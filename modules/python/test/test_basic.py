import visp
from visp.core import ArrayDouble2D, RotationMatrix, Matrix, HomogeneousMatrix, PoseVector

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

def test_representations_not_writable():
  # Test that some classes have non writable numpy arrays
  R = RotationMatrix()
  R_np = np.array(R, copy=False)
  with pytest.raises(ValueError):
    R_np[0, 0] = 1
  # with pytest.raises(ValueError):
  #   R.numpy()[:1] = 0
  with pytest.raises(ValueError):
    row = R[0]
    row[0] = 1

  T = HomogeneousMatrix()
  T_np = np.array(T, copy=False)
  with pytest.raises(ValueError):
    T_np[0, 0] = 1
  # q = visp.core.QuaternionVector()
  # q_np = np.array(q, copy=False)
  # with pytest.raises(ValueError):
  #   q_np[0] = 1

def test_numpy_constructor():
  n_invalid = np.array([1, 2, 3])
  with pytest.raises(RuntimeError):
    a = ArrayDouble2D(n_invalid)
  n_valid = np.array([[1, 2, 3], [4, 5, 6]])
  a = ArrayDouble2D(n_valid)

  assert np.all(np.equal(a.numpy(), n_valid))

def test_numpy_conversion_and_back():
  a = ArrayDouble2D(10, 10, 2.0)
  a_np = a.numpy().copy()
  a2 = ArrayDouble2D(a_np)
  mat = Matrix(a_np)

  for i in range(a.getRows()):
    for j in range(a.getCols()):
      assert a[i, j] == a_np[i, j]
      assert a[i, j] == a2[i,  j]
      assert mat[i, j] == a[i, j]

def test_indexing_array2D():
  a_np = np.asarray([[i for _ in range(10)] for i in range(10)])
  a = ArrayDouble2D(a_np)
  col = list(range(10))
  for i in range(a.getRows()):
    assert np.all(a[i] == float(i))
    assert np.all(a[-i - 1] == float(a.getRows() - i - 1))
    assert np.all(a[:, i] == col)
    assert np.all(a[:, -i - 1] == col)

def test_index_array2D_is_not_copy():
  a = ArrayDouble2D(5, 5, 1.0)
  first_row_view = a[0]
  first_row_view[0] = 0.0
  assert a[0, 0] == 0.0
  sub_matrix = a[1:3, 1:3]
  sub_matrix[0, 0] = 0.0
  assert a[1, 1] == 0.0
  col = a[:, -1]
  col[0] = 0.0
  assert a[0, -1] == 0.0


def test_rotation_repr_can_be_defined_by_hand():
  R = RotationMatrix()