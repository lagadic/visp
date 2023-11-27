List of todos
======================

What remains to be done

Changes to ViSP
------------------

* Write initTracking for vpKltOpencv taking a vpImage<unsigned char> as input. Ignore setInitialGuess.


Code generation
-------------------

* There is an issue with vpFeatureMomentDatabse::get and vpMomentDatabase::get, ignored for now => a tester
* n ary operators
* Exclude get operators for vpArray2D ?
* Parse subnamespaces

  * Classes in subnamespaces are ignored

* Keep alive for numpy interfaces
* Keep alive very probably for mbt
* How should we handle parameters coming from external APIs ? e.g. realsense2, PCL. Can we interact with other bindings such as of opencv's
* Reimplement a framegrabber tutorial in python, with matplotlib
* Test return policy for lvalue references (automatic is copy, so this is problematic)
* Add parameters in config for:

  * Return policy (feature moments database)
  * Keep alive
  * GIL scope

* Add callback for before_module and after_module so that we can define additional bindings by hand in the module. This is already done per class
* Add a way to replace a default method binding with a custom one (What about doc?)

Documentation
----------------
* Generate documentation for:

  * Enums
  * Functions in namespaces etc.
  * In classes

    * Constructors
    * Operators

* Reference python types in Documentation
* Prefer Python examples instead of C++ ones ?


To be written:
* Specific changes from C++ to Python API
* Documentation for the overall workflow of the bindings generation
* In code documentation for the generator
* Document config files

* Failure cases

  *  If you have this error:
      error: invalid new-expression of abstract class type ‘vpTemplateTrackerMI’
      return new Class{std::forward<Args>(args)...};
      In file included from /home/sfelton/software/visp_build/modules/python/bindings/src/tt_mi.cpp:13:0:
      /home/sfelton/software/visp-sfelton/modules/tracker/tt_mi/include/visp3/tt_mi/vpTemplateTrackerMI.h:46:19: note:   because the following virtual functions are pure within ‘vpTemplateTrackerMI’:
      class VISP_EXPORT vpTemplateTrackerMI : public vpTemplateTracker
    You should define the class (here vpTemplaterMI) as pure virtual in the config file (via the flag is_virtual)
    This error occurs because some methods are defined as pure virtual in a parent class and are not defined in the class this class: Pure virtual class detection does not look in the class hierarchy but only at the present class




Python side
-----------------
* Testing

  * Test numpy arrays, partially done
  * Test specific methods with specific returns
  * Test keep alive if possible ?

* Generate some examples

  * Tracking (megapose/mbt)
  * Frame grabbing
  * UI

* Add python sources to visp package

  * Matplotlib based plotter
