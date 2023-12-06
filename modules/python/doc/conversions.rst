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
they are a foreign concept to all the other Python libraries.

For most scientific computing libraries, the standard data representation is based on `NumPy <https://numpy.org/>`_.
Since most libraries will accept and manipulate these arrays, ViSP provides conversion functions.


From ViSP to NumPy
-----------------------------------------


Obtaining a view of a ViSP object
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To reinterpret a supported ViSP object as a Numpy array, use either:

.. doctest::

  >>> from visp.core import ColVector
  >>> import numpy as np

  >>> list_representation = [i for i in range(3)]
  >>> vec = ColVector(list_representation) # Initialize a 3 vector from a list
  >>> np_vec = vec.numpy() # A 1D numpy array of size 3

  >>> np.all(np_vec == list_representation)
  True

  >>> vec *= 2.0
  >>> np.all(np_vec == list_representation)
  False

or

.. doctest::

  >>> from visp.core import ColVector
  >>> import numpy as np

  >>> list_representation = [i for i in range(3)]
  >>> vec = ColVector(list_representation) # Initialize a 3 vector from a list
  >>> np_vec = np.array(vec, copy=False) # A 1D numpy array of size 3

  >>> np.all(np_vec == list_representation)
  True

  >>> vec *= 2.0 # Modifying the ViSP vector modifies the NumPy view
  >>> np.all(np_vec == list_representation)
  False

To obtain a copy of the ViSP representation you can simply use:

.. doctest::

  >>> from visp.core import ColVector
  >>> import numpy as np

  >>> vec = ColVector(3, 0)
  >>> np_vec = vec.numpy().copy() # or np.array(vec, copy=True)
  >>> np_vec[0] = 1

  >>> np_vec[0] == vec[0]
  False

Note that with these methods, some ViSP objects cannot be modified.
That is the case for :py:class:`visp.core.HomogeneousMatrix` and :py:class:`visp.core.RotationMatrix`, where an undesired modification
may lead to an invalid representation (Such as a rotation matrix not conserving its properties)

Thus, this code will not work:

.. doctest::
  :options: +IGNORE_EXCEPTION_DETAIL

  >>> from visp.core import RotationMatrix, HomogeneousMatrix
  >>> import numpy as np

  >>> R = RotationMatrix()
  >>> R.numpy()[0, 1] = 1.0
  Traceback (most recent call last):
   File "<stdin>", line 1, in <module>
  ValueError: assignment destination is read-only

  >>> T = HomogeneousMatrix()
  >>> R.numpy()[0, 1] = 1.0
  Traceback (most recent call last):
   File "<stdin>", line 1, in <module>
  ValueError: assignment destination is read-only








Potential issues
--------------------
