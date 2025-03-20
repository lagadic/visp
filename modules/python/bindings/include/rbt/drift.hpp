
#ifndef VISP_PYTHON_RBT_DRIFT_HPP
#define VISP_PYTHON_RBT_DRIFT_HPP

#include <visp3/rbt/vpRBDriftDetector.h>
#include <pybind11/pybind11.h>



class PyDriftDetector : public vpRBDriftDetector
{
  using vpRBDriftDetector::vpRBDriftDetector;

  virtual std::string explainAsString() const = 0;

};

class TrampolineRBDriftDetector : public PyDriftDetector
{
public:
  using PyDriftDetector::PyDriftDetector;

  TrampolineRBDriftDetector() : PyDriftDetector() { }

  virtual void update(const vpRBFeatureTrackerInput &previousFrame, const vpRBFeatureTrackerInput &frame, const vpHomogeneousMatrix &cTo, const vpHomogeneousMatrix &cprevTo)
  {
    pybind11::gil_scoped_acquire gil;  // Acquire the GIL while in this scope.
    // Try to look up the overridden method on the Python side.
    pybind11::function override = pybind11::get_override(this, "update");
    if (override) {  // method is found
      // Pybind seems to copy the frames, so we pass the pointers
      override(&previousFrame, &frame, &cTo, &cprevTo);
    }
  }

  virtual double getScore() const VP_OVERRIDE
  {
    PYBIND11_OVERRIDE_PURE(
      double,               /* Return type */
      vpRBDriftDetector,    /* Parent class */
      getScore,             /* Name of function in C++ (must match Python name) */
      );
  }

  virtual bool hasDiverged() const VP_OVERRIDE
  {
    PYBIND11_OVERRIDE_PURE(
      double,                 /* Return type */
      vpRBDriftDetector,      /* Parent class */
      hasDiverged,            /* Name of function in C++ (must match Python name) */
      );
  }

  virtual void display(const vpImage<vpRGBa> &I) VP_OVERRIDE
  {
    PYBIND11_OVERRIDE_PURE(
      void,                   /* Return type */
      vpRBDriftDetector,      /* Parent class */
      display,                /* Name of function in C++ (must match Python name) */
      I
    );
  }


#if defined(VISP_HAVE_NLOHMANN_JSON)
  void loadJsonConfiguration(const nlohmann::json &) VP_OVERRIDE
  {

  }

  nlohmann::ordered_json explain() const VP_OVERRIDE
  {
    std::string s = explainAsString();
    nlohmann::ordered_json j = nlohmann::json::parse(s);
    return j;

  }

  std::string explainAsString() const VP_OVERRIDE
  {
    PYBIND11_OVERRIDE_PURE(
      std::string,                   /* Return type */
      PyDriftDetector,      /* Parent class */
      explainAsString,                /* Name of function in C++ (must match Python name) */
      );
  }




#endif

};



#endif
