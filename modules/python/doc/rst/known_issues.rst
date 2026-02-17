.. _Known issues:

Known issues
======================

We are aware of some remaining issues.
If you encounter another problem, please file an issue on Github.


Usability
--------------------

No implicit conversion from ViSP types to Numpy
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Numpy array cannot be implicitly converted to a ViSP representation when calling a ViSP function.


ViSP 3rd party types (such as cv::Mat) cannot be used from Python
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

We do not interface with other bindings (as it is not trivial and may require specific Pybind ABI), and we do not wrap third party types.
Thus, alternatives must be provided by hand into the ViSP API (or wrapped through custom bindings) so that the functionalities can be used from Python.

Cannot inherit from a ViSP class in Python
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Right now, it is not possible to inherit from a ViSP class with a Python class. Virtual methods cannot be overridden.
To remedy this, trampoline classes should be implemented into the generator, either fully automated (but that is complex)
or by providing the trampoline by hand and adding a way to reference the trampoline class in the configuration file.
