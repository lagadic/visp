List of todos
======================

* There is an issue when indexing readonly arrays such as HomogeneousMatrix or RotationMatrix
* Immutable (in python) parameters that are passed by reference are not modified. This includes:

  * ints, floats, double
  * std::string and char arrays
  * To remedy this, they should be returned as new values by defining the fn as a lambda by changing

* Std containers are not modified, same as immutable parameters
* How should we handle parameters coming from external APIs ? e.g. realsense2, PCL. Can we interact with other bindings such as of opencv's
* Reimplement a framegrabber tutorial in python, with matplotlib