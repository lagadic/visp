Python specific functions
==============================

To either make code more pythonic or help improve performance, some functions and helpers have been defined.

To add other custom functionalities :ref:`Custom binding`

NumPy-like indexing
---------------------

In Python ViSP data types now support numpy-like indexing, and methods like slicing and iterating on values

To read values, rows and columns of a Matrix, you can use:

.. testcode::

  from visp.core import Matrix

  m = Matrix(2, 3, 1.0)
  print(m[0, 0])
  print(m[0]) # First row
  print(m[:, 0]) # First column


.. testoutput::

  1.0
  [1. 1. 1.]
  [1. 1.]




Core module
----------------------

* :py:class:`~visp.core.PixelMeterConversion` and :py:class:`~visp.core.MeterPixelConversion`
both have a vectorised implementation of :code:`convertPoint`, called :code:`convertPoints`, accepting NumPy arrays
* :py:class:`~visp.mbt.MbGenericTracker` as a reworked version of :py:meth:`visp.mbt.MbGenericTracker.track`, taking as inputs
maps of color images and of numpy representations (of shape H x W x 3) of the point clouds.
