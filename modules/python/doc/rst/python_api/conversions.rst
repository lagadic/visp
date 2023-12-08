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


NumPy <-> ViSP
-----------------------------------------



Mapping between NumPy arrays and ViSP types
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^




Acquiring a view of a ViSP object
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

You can view

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


Obtaining a copy of the data
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To obtain a copy of the ViSP representation you can simply use:

.. doctest::

  >>> from visp.core import ColVector
  >>> import numpy as np

  >>> vec = ColVector(3, 0)
  >>> np_vec = vec.numpy().copy() # or np.array(vec, copy=True)
  >>> np_vec[0] = 1

  >>> np_vec[0] == vec[0]
  False


Keep in mind that it may be preferable to use a copy of the data, especially if you are using both numpy and ViSP representations for different tasks at the same time

For instance, the following code will lead to an undesired behaviour:

.. testcode::

  from visp.core import ColVector
  import numpy as np

  def compute_velocity(velocity) -> None:
    # Dummy function to illustrate in place
    velocity *= 2.0 # This code modifies the content of velocity

  velocity = ColVector(6, 0.1)
  iteration = 0
  # Store the velocities in a list
  log_data = []

  # Servoing loop
  while iteration < 10:
    compute_velocity(v)
    log_data.append(v.numpy())
    iteration += 1

  # Do some logging...
  print(log_data[0])
  print(log_data[-1])

.. test_output::

  array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
  array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])


Although we're multiplying the velocity by 2 at each iteration,
we can see that we have the same values for the first and last iterations.

In essence, this is because while we store 10 different NumPy arrays, they all share the same underlying storage.
This storage is, at each iteration, modified by the :python:`compute_velocity` function.

To remedy, you can either:

* Make a copy of the NumPy array at every iteration before storing it in the list
* Change the :python:`compute_velocity` to return a new :py:class:`visp.core.ColVector`



Potential issues
--------------------
