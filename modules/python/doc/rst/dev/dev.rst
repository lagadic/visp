.. _Development guide:

Modifying and contributing to the bindings
============================================

.. toctree::
    :glob:
    :maxdepth: 2

    how.rst
    config.rst
    custom_bindings.rst
    python_side.rst



Remaining work
-----------------------

In this section, we list some remaining issues or work to be done.


Changes to ViSP C++ API
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Write initTracking for vpKltOpencv taking a vpImage<unsigned char> as input. Ignore setInitialGuess.

Code generation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* n-ary operators are not generated
* Matrix multiplication should be done through the @ operator (__matmul\__)
* Get operators for vpArray2D and the subclasses should be ignored, as they are reimplemented through custom bindings
* Classes that are not in the top level namespace are ignored.
* Inner classes are also ignored
* The default return policy for references is to copy, which is probably not the expected usage. ViSP sometimes returns references to STL containers, which have to be copied to Python
* Add parameters in config for:

  * GIL scope

* Add callback for before_module and after_module so that we can define additional bindings by hand in the module. This is already done per class

Documentation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Generate documentation for:

  * Functions in namespaces etc.

* Reference python types in Documentation
* Prefer Python examples instead of C++ ones ?

To be written:
* Documentation for the overall workflow of the bindings generation


Python side
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
* UI

* Add python sources to visp package

  * Matplotlib based plotter



Errors when generating bindings
-------------------------------------

When modifying the bindings, you may encounter errors.

Here is a very non-exhaustive list of errors.

If you encounter a compilation error, make sure to first try rebuilding after cleaning the CMake cache
Pybind did generate problems (an error at the pybind include line) that were solved like this.

Static and member methods have the same name
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If, when importing visp in python, you encounter this message:

  ImportError: overloading a method with both static and instance methods is not supported; error while attempting to bind instance method visp.xxx() -> None

Then it means that a class has both a static method and a member method with the same name. You should :ref:`rename either one through the config files <Function options>`.

Abstract class not detected
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If you have this error:

  error: invalid new-expression of abstract class type ‘vpTemplateTrackerMI’
  return new Class{std::forward<Args>(args)...};
  In file included from /home/visp_ws/visp_build/modules/python/bindings/src/tt_mi.cpp:13:0:
  /home/visp_ws/visp/modules/tracker/tt_mi/include/visp3/tt_mi/vpTemplateTrackerMI.h:46:19: note:   because the following virtual functions are pure within ‘vpTemplateTrackerMI’:
  class VISP_EXPORT vpTemplateTrackerMI : public vpTemplateTracker

You should define the class (here vpTemplaterMI) as pure virtual in the config file (via the flag is_virtual).
This error occurs because some methods are defined as pure virtual in a parent class and are not defined in the class this class: Pure virtual class detection does not look in the class hierarchy but only at the present class.
