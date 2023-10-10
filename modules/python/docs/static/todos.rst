List of todos
======================

What remains to be done

Code generation
---------------

* There is an issue when indexing readonly arrays such as HomogeneousMatrix or RotationMatrix
* Unary and n ary operators
* Exclude get operators for vpArray2D ?
* Parse subnamespaces
  * Classes in subnamespaces are ignored
* Keep alive for numpy interfaces
* How should we handle parameters coming from external APIs ? e.g. realsense2, PCL. Can we interact with other bindings such as of opencv's
* Reimplement a framegrabber tutorial in python, with matplotlib


Documentation
----------------

* In classes, generate documentation for: constructors, operators
* Reference python types in Documentation
* Prefer Python examples instead of C++ ones ?
* Generate documentation for functions that are not in classes
* Generate documentation for enums
* Allow passing py::gil_scope_release etc to function from config
* Same for keep_alive (feature moments database)
* Export Tracking modules
* Add callback for before_module and after_module so that we can define additional bindings by hand in the module. This is already done per class
* Add a way to replace a default method binding with a custom one (What about doc?)

To be written:
* Specific changes from C++ to Python API
* Documentation for the overall workflow of the bindings generation
* In code documentation for the generator
* Document config files


Packaging
------------------

* Root CMake

  * Build last
  * Build after doc if doc can be generated
  * Correspondance between headers and modules
  * Copy to binary dir
  * Compile in binary dir

* Package generator to install as an editable
* Automatic version

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
