Using ViSP with other libraries
===============================================

ViSP provides multiple types to manipulate mathematical objects, such as:

* Vectors

  * :py:class:`visp.core.ColVector`
  * :py:class:`visp.core.RowVector`
  * :py:class:`visp.core.ThetaUVector`

* Matrices

  * :py:class:`visp.core.Matrix`
  * :py:class:`visp.core.RotationMatrix`
  * :py:class:`visp.core.HomogeneousMatrix`


While these representations should allow you to work with all the ViSP functions,
they are foreign to all the other Python libraries.

For most scientific computing libraries, the standard data representation is based on `NumPy <https://numpy.org/>`_.
Since most libraries will accept and manipulate these arrays, ViSP provides conversion functions.


NumPy ↔ ViSP
-----------------------------------------



Mapping between NumPy arrays and ViSP types
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Here, we give the list of all supported NumPy-convertible types

.. list-table:: Core math types
   :header-rows: 1

   * - Python type
     - NumPy Shape and dtype
   * - :py:class:`visp.core.Matrix`
     - (N, M), np.float64
   * - :py:class:`visp.core.ColVector`
     - (M,), np.float64
   * - :py:class:`visp.core.RowVector`
     - (N,), np.float64
   * - :py:class:`visp.core.RotationMatrix`
     - (3, 3), np.float64
   * - :py:class:`visp.core.HomogeneousMatrix`
     - (4, 4), np.float64
   * - :py:class:`visp.core.ThetaUVector`
     - (3,), np.float64

.. list-table:: Core image types
   :header-rows: 1

   * - C++ type
     - Python type
     - NumPy Shape and dtype
   * - :code:`vpImage<unsigned char>`
     - :py:class:`visp.core.ImageGray`
     - (H, W), np.uint8
   * - :code:`vpImage<uint16_t>`
     - :py:class:`visp.core.ImageUInt16`
     - (H, W), np.uint16
   * - :code:`vpImage<vpRGBa>`
     - :py:class:`visp.core.ImageRGBa`
     - (H, W, 4), np.uint8
   * - :code:`vpImage<vpRGBf>`
     - :py:class:`visp.core.ImageRGBf`
     - (H, W, 3), np.float32



Acquiring a view of a ViSP object
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

It is possible to view a ViSP object as a NumPy array. When using a view, changes made to one representation is reflected in the other.

See `the NumPy documentation <https://numpy.org/doc/stable/user/basics.copies.html>`_ for more information.

To reinterpret a supported ViSP object as a Numpy array, use either:

.. testcode::

  from visp.core import ColVector
  import numpy as np

  list_representation = [i for i in range(3)]
  vec = ColVector(list_representation) # Initialize a 3 vector from a list
  np_vec = vec.numpy() # A 1D numpy array of size 3

  print(np.all(np_vec == list_representation))


  vec *= 2.0
  print(np.all(np_vec == list_representation))

.. testoutput::

  True
  False


or

.. testcode::

  from visp.core import ColVector
  import numpy as np

  list_representation = [i for i in range(3)]
  vec = ColVector(list_representation) # Initialize a 3 vector from a list
  np_vec = np.array(vec, copy=False) # A 1D numpy array of size 3

  print(np.all(np_vec == list_representation))
  # Modifying the ViSP vector modifies the NumPy view
  vec *= 2.0
  print(np.all(np_vec == list_representation))

  # Modifying the NumPy array modifies the ViSP object
  np_vec[:2] = 0.0
  print(vec[0] == 0.0 and vec[1] == 0.0)

.. testoutput::

  True
  False
  True


Note that with these methods, some ViSP objects cannot be modified.
That is the case for :py:class:`visp.core.HomogeneousMatrix` and :py:class:`visp.core.RotationMatrix`, where an undesired modification
may lead to an invalid representation (Such as a rotation matrix not conserving its properties)

Thus, this code will not work:

.. testcode::

  from visp.core import RotationMatrix, HomogeneousMatrix
  import numpy as np

  R = RotationMatrix()
  R.numpy()[0, 1] = 1.0

  T = HomogeneousMatrix()
  T.numpy()[0, 1] = 1.0

.. testoutput::
  :options: +IGNORE_EXCEPTION_DETAIL

  Traceback (most recent call last):
   File "<stdin>", line 1, in <module>
  ValueError: assignment destination is read-only
  Traceback (most recent call last):
   File "<stdin>", line 1, in <module>
  ValueError: assignment destination is read-only


Copying to a NumPy array
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To obtain a copy of the ViSP representation you can simply use:

.. testcode::

  from visp.core import ColVector
  import numpy as np

  vec = ColVector(3, 0)
  np_vec = vec.numpy().copy() # or np.array(vec, copy=True)
  np_vec[0] = 1

  print(np_vec[0] == vec[0])

.. testoutput::

  False


Keep in mind that it may be preferable to use a copy of the data, especially if you are using both numpy and ViSP representations for different tasks at the same time

For instance, the following code will lead to an undesired behaviour:

.. testcode::

  from visp.core import ColVector
  import numpy as np

  def compute_velocity(velocity: ColVector) -> None:
    # Dummy function to illustrate in place ops
    velocity *= 2.0 # This code modifies the content of velocity

  velocity = ColVector(6, 1.0)
  iteration = 0
  # Store the velocities in a list
  log_data = []

  # Servoing loop
  while iteration < 10:
    compute_velocity(velocity)
    log_data.append(velocity.numpy())
    iteration += 1

  # Do some logging...
  print(log_data[0])
  print(log_data[-1])

.. testoutput::

  [1024. 1024. 1024. 1024. 1024. 1024.]
  [1024. 1024. 1024. 1024. 1024. 1024.]


Although we're multiplying the velocity by 2 at each iteration,
we can see that we have the same values for the first and last iterations.


.. warning::

  In essence, this is because while we store 10 different NumPy arrays, they all share the same underlying storage.
  This storage is, at each iteration, modified by the :python:`compute_velocity` function.

.. note::

  To remedy this, you can either:

  * Make a copy of the NumPy array at every iteration before storing it in the list :python:`log_data.append(velocity.numpy().copy())`
  * Change the :python:`compute_velocity` to return a new :py:class:`visp.core.ColVector`


Building a ViSP object from a NumPy array
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

In the above section, we have shown how to convert a ViSP representation to a NumPy array.

To perform the inverse operation, a custom constructor is defined for each class that allows the Numpy -> ViSP conversion.

This constructor performs a **copy** of the NumPy data into the newly created ViSP object.

For instance, to build a new matrix


.. testsetup::

  from visp.core import Matrix
  import numpy as np


.. testcode::

  random_mat = np.random.rand(10, 10) # 10 x 10 random matrix

  mat = Matrix(random_mat)
  print(mat.getRows(), mat.getCols())
