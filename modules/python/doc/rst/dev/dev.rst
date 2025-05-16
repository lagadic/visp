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



Errors and issues when generating bindings
==========================================

When modifying the bindings, you may encounter errors.

Here is a very non-exhaustive list of errors.

If you encounter a compilation error, make sure to first try rebuilding after cleaning the CMake cache
Pybind did generate problems (an error at the pybind include line) that were solved like this.

Static and member methods have the same name
----------------------------------------------

If, when importing visp in python, you encounter this message:

::

  ImportError: overloading a method with both static and instance methods is not supported; error while attempting to bind instance method visp.xxx() -> None

Then it means that a class has both a static method and a member method with the same name. You should :ref:`rename either one through the config files <Function options>`.

Abstract class not detected
----------------------------------------------

If you have this error:

::

  error: invalid new-expression of abstract class type ‘vpTemplateTrackerMI’
  return new Class{std::forward<Args>(args)...};
  In file included from $VISP_WS/visp-build/modules/python/bindings/src/tt_mi.cpp:13:0:
  $VISP_WS/visp/modules/tracker/tt_mi/include/visp3/tt_mi/vpTemplateTrackerMI.h:46:19: note:   because the following virtual functions are pure within ‘vpTemplateTrackerMI’:
  class VISP_EXPORT vpTemplateTrackerMI : public vpTemplateTracker

You should define the class (here vpTemplaterMI) as pure virtual in the `tt_mi.json` config file (via the flag `is_virtual`).
This error occurs because some methods are defined as pure virtual in a parent class.
Pure virtual class detection does not look in the class hierarchy but only at the present class.


Template errors
----------------------------------------------


If you have an issue that looks like:

::

  Consolidate compiler generated dependencies of target _visp
  [ 97%] Building CXX object modules/python/bindings/CMakeFiles/_visp.dir/src/core.cpp.o
  [ 97%] Building CXX object modules/python/bindings/CMakeFiles/_visp.dir/src/robot.cpp.o
  In file included from /usr/include/c++/11/bits/move.h:57,
                  from /usr/include/c++/11/bits/stl_pair.h:59,
                  from /usr/include/c++/11/bits/stl_algobase.h:64,
                  from /usr/include/c++/11/bits/specfun.h:45,
                  from /usr/include/c++/11/cmath:1935,
                  from /usr/include/c++/11/math.h:36,
                  from /home/sfelton/miniconda3/envs/wrapper3.9/include/python3.9/pyport.h:205,
                  from /home/sfelton/miniconda3/envs/wrapper3.9/include/python3.9/Python.h:50,
                  from /home/sfelton/.local/include/pybind11/detail/common.h:266,
                  from /home/sfelton/.local/include/pybind11/attr.h:13,
                  from /home/sfelton/.local/include/pybind11/detail/class.h:12,
                  from /home/sfelton/.local/include/pybind11/pybind11.h:13,
                  from /home/sfelton/software/visp_build/modules/python/bindings/src/robot.cpp:3:
  /usr/include/c++/11/type_traits: **In instantiation of ‘struct std::is_move_constructible<vpImage<double> >’:**
  /usr/include/c++/11/type_traits:152:12:   required from ‘struct std::__and_<std::is_move_constructible<vpImage<double> >, std::is_move_assignable<vpImage<double> > >’
  /usr/include/c++/11/type_traits:157:12:   required from ‘struct std::__and_<std::__not_<std::__is_tuple_like<vpImage<double> > >, std::is_move_constructible<vpImage<double> >, std::is_move_assignable<vpImage<double> > >’
  /usr/include/c++/11/type_traits:2209:11:   required by substitution of ‘template<class ... _Cond> using _Require = std::__enable_if_t<std::__and_< <template-parameter-1-1> >::value> [with _Cond = {std::__not_<std::__is_tuple_like<vpImage<double> > >, std::is_move_constructible<vpImage<double> >, std::is_move_assignable<vpImage<double> >}]’
  /usr/include/c++/11/bits/move.h:196:5:   required by substitution of ‘template<class _Tp> std::_Require<std::__not_<std::__is_tuple_like<_Tp> >, std::is_move_constructible<_Tp>, std::is_move_assignable<_Tp> > std::swap(_Tp&, _Tp&) [with _Tp = vpImage<double>]’
  /home/sfelton/software/visp-sfelton/modules/core/include/visp3/core/vpImage.h:341:15:   required from ‘class vpImage<double>’
  /home/sfelton/software/visp-sfelton/modules/core/include/visp3/core/vpImage.h:369:17:   required from here
  /usr/include/c++/11/type_traits:1010:52: error: static assertion failed: template argument must be a complete class or an unbounded array
  1010 |       **static_assert(std::__is_complete_or_unbounded(__type_identity<_Tp>{}),**

You should delete the files in `modules/python/` of the build directory.
