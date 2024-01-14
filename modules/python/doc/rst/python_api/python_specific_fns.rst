Python specific functions
==============================

To either make code more pythonic or help improve performance, some functions and helpers have been defined.

To add other custom functionalities :ref:`Custom binding`.


Core module
----------------------

* :py:class:`~visp.core.PixelMeterConversion` and :py:class:`~visp.core.MeterPixelConversion` both
have a vectorised implementation of :code:`convertPoint`, called :code:`convertPoints`, accepting NumPy arrays


MBT module
-----------------------

* :py:class:`~visp.mbt.MbGenericTracker` as a reworked version of :py:meth:`visp.mbt.MbGenericTracker.track`, taking as inputs
maps of color images and of numpy representations (of shape H x W x 3) of the point clouds.
