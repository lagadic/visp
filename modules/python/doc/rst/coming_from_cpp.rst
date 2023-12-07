.. _CPP guide:

Differences with C++ ViSP
==============================

In this section, we highlight the differences with writing ViSP code in C++.

Module structure
-----------------------------

The overall module structure remains the same.
What was, in C++, referred as :code:`visp3/core/*` can now be accessed as :python:`visp.core.*` in Python.
Note that before this works in Python, you need to :python:`import visp`


Naming convention
-----------------------------

In C++, each class has the prefix `vp`. In Python, this prefix has been dropped as imports can be aliased and full names can be used.

.. testcode::

  import visp.core
  from visp.core import Matrix as vpMatrix # if the name clashes with another lib

  m = vpMatrix()
  vec = visp.core.ColVector(10) # Use the full name, explicit visp use



Importing a class
----------------------------

The syntax to import a ViSP class into the current scope is different.

In C++, including a header file pulls everything in it (functions and classes).


In the ViSP Python API, no distinction is made between the different headers: everything is at the same level in the package hierarchy.
In Python, you can import a single symbol.

Thus, if a single header contains two symbols in ViSP, you will need to import both on the Python side.

Below, the difference in syntax between C++ and Python on imports is illustrated:

.. tab-set::

    .. tab-item:: C++
        :sync: cpp

        .. code-block:: cpp

          #include <visp3/core/vpImageConvert.h>
          #include <visp3/core/vpColVector.h>
          #include <visp3/core/vpMatrix.h>


    .. tab-item:: Python
        :sync: python

        .. testcode::

          from visp.core import ImageConvert
          from visp.core import ColVector, Matrix # Grouping relevant imports


You can also import everything from a single submodule:

.. tab-set::

    .. tab-item:: C++
        :sync: cpp

        .. code-block:: cpp

          #include <visp3/visp_core.h>


    .. tab-item:: Python
        :sync: python

        .. testcode::

          from visp.core import *


Changes in function parameters
--------------------------------------

For some functions, the Python API differs from the C++ one, mainly in the input arguments and return type.

Due to python considering basic types as immutable, it is no longer possible to modify them passing their reference to a function call.

Thus, we have made the choice to modify the functions such that these immutable types, if they are modified, are returned along with the original type.

This encompasses other types, such as lists (std::vector), and dictionaries (maps)


Naively translating the use of :code:`convertPoint` from C++:

.. testcode:: error_args

  from visp.core import PixelMeterConversion, CameraParameters
  cam = CameraParameters(600, 600, 320, 240)
  u, v = 240, 320
  x, y = 0, 0
  PixelMeterConversion.convertPoint(cam, u, v, x, y) # WRONG: C++-like version, using references to modify x and y

Would lead to an error such as:

.. testoutput:: error_args
  :options: -ELLIPSIS, +NORMALIZE_WHITESPACE, +IGNORE_EXCEPTION_DETAIL

  Traceback (most recent call last):
    File "<stdin>", line 1, in <module>
  TypeError: convertPoint(): incompatible function arguments. The following argument types are supported:
    1. (cam: _visp.core.CameraParameters, u: float, v: float) -> Tuple[float, float]
    2. (cam: _visp.core.CameraParameters, iP: _visp.core.ImagePoint) -> Tuple[float, float]

  Invoked with: Camera parameters for perspective projection without distortion:
    px = 600	 py = 600
    u0 = 320	 v0 = 240
  , 240, 320, 0, 0

Because this function has been modified to return a tuple of :code:`Tuple[float, float]` (the x and y values).
The x and y arguments are no longer accepted, as they are output only.

Thus, the correct function call is:

.. testcode:: error_args

  from visp.core import PixelMeterConversion, CameraParameters
  cam = CameraParameters(600, 600, 320, 240)
  u, v = 240, 320
  x, y = PixelMeterConversion.convertPoint(cam, u, v)


If you have such errors, it is recommended that you look at the Python :ref:`API reference` for the function and look at its signature.



.. tab-set::

    .. tab-item:: C++
        :sync: cpp

        .. code-block:: cpp

          #include <visp3/visp_core.h>


    .. tab-item:: Python
        :sync: python

        .. testcode::

          from visp.core import *
