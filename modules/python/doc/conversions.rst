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

For most scientific computing libraries, the standard data representation is based on`NumPy <https://numpy.org/>`_.
Since most libraries will accept and manipulate these arrays, ViSP provides conversion functions.

To reinterpret a supported ViSP object as a Numpy array, use either:


.. testcode::

  from visp.core import ColVector
  import numpy as np

  list_representation = [i for i in range(3)]
  vec = ColVector(list_representation) # Initialize a 3 vector from a list
  np_vec = vec.numpy() # A 1D numpy array of size 3

  print(np.all(np_vec == list_representation))

.. testoutput::

   True








Potential issues
--------------------
