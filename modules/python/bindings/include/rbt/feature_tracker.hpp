
#ifndef VISP_PYTHON_RBT_FEATURE_TRACKER_HPP
#define VISP_PYTHON_RBT_FEATURE_TRACKER_HPP

#include <visp3/rbt/vpRBFeatureTracker.h>
#include <pybind11/pybind11.h>


class TrampolineRBFeatureTracker : public vpRBFeatureTracker
{
public:
  using vpRBFeatureTracker::vpRBFeatureTracker;

  TrampolineRBFeatureTracker() : vpRBFeatureTracker() { }


  virtual bool requiresRGB() const VP_OVERRIDE
  {
    PYBIND11_OVERRIDE_PURE(
      bool,           /* Return type */
      vpRBFeatureTracker,     /* Parent class */
      requiresRGB,        /* Name of function in C++ (must match Python name) */
      );
  }
  virtual bool requiresDepth() const VP_OVERRIDE
  {
    PYBIND11_OVERRIDE_PURE(
      bool,           /* Return type */
      vpRBFeatureTracker,     /* Parent class */
      requiresDepth       /* Name of function in C++ (must match Python name) */
    );
  }
  virtual bool requiresSilhouetteCandidates() const VP_OVERRIDE
  {
    PYBIND11_OVERRIDE_PURE(
      bool,           /* Return type */
      vpRBFeatureTracker,     /* Parent class */
      requiresSilhouetteCandidates      /* Name of function in C++ (must match Python name) */
    );
  }
  virtual void onTrackingIterStart() VP_OVERRIDE
  {
    PYBIND11_OVERRIDE_PURE(
      void,           /* Return type */
      vpRBFeatureTracker,     /* Parent class */
      onTrackingIterStart        /* Name of function in C++ (must match Python name) */
    );
  }
  virtual void onTrackingIterEnd() VP_OVERRIDE
  {
    PYBIND11_OVERRIDE_PURE(
      void,           /* Return type */
      vpRBFeatureTracker,     /* Parent class */
      onTrackingIterEnd      /* Name of function in C++ (must match Python name) */
    );
  }
  virtual void extractFeatures(const vpRBFeatureTrackerInput &frame, const vpRBFeatureTrackerInput &previousFrame, const vpHomogeneousMatrix &cMo)
    VP_OVERRIDE
  {
    PYBIND11_OVERRIDE_PURE(
      void,           /* Return type */
      vpRBFeatureTracker,     /* Parent class */
      extractFeatures,        /* Name of function in C++ (must match Python name) */
      frame, previousFrame, cMo
    );
  }
    virtual void trackFeatures(const vpRBFeatureTrackerInput &frame, const vpRBFeatureTrackerInput &previousFrame, const vpHomogeneousMatrix &cMo)
    VP_OVERRIDE
  {
    PYBIND11_OVERRIDE_PURE(
      void,           /* Return type */
      vpRBFeatureTracker,     /* Parent class */
      trackFeatures,        /* Name of function in C++ (must match Python name) */
      frame, previousFrame, cMo
    );
  }
    virtual void initVVS(const vpRBFeatureTrackerInput &frame, const vpRBFeatureTrackerInput &previousFrame, const vpHomogeneousMatrix &cMo) VP_OVERRIDE
  {
    PYBIND11_OVERRIDE_PURE(
      void,           /* Return type */
      vpRBFeatureTracker,     /* Parent class */
      initVVS,        /* Name of function in C++ (must match Python name) */
      frame, previousFrame, cMo
    );
  }
  virtual void computeVVSIter(const vpRBFeatureTrackerInput &frame, const vpHomogeneousMatrix &cMo, unsigned int iteration) VP_OVERRIDE
  {
    PYBIND11_OVERRIDE_PURE(
      void,           /* Return type */
      vpRBFeatureTracker,     /* Parent class */
      computeVVSIter,        /* Name of function in C++ (must match Python name) */
      frame, cMo, iteration
    );
  }
  virtual void display(const vpCameraParameters &cam, const vpImage<unsigned char> &I, const vpImage<vpRGBa> &IRGB, const vpImage<unsigned char> &depth, const vpRBFeatureDisplayType type) const VP_OVERRIDE
  {
    PYBIND11_OVERRIDE_PURE(
      void,           /* Return type */
      vpRBFeatureTracker,     /* Parent class */
      display,        /* Name of function in C++ (must match Python name) */
      cam, I, IRGB, depth, type
    );
  }
  virtual const vpMatrix getCovariance() const VP_OVERRIDE
  {
    PYBIND11_OVERRIDE(
      vpMatrix,           /* Return type */
      vpRBFeatureTracker,     /* Parent class */
      getCovariance        /* Name of function in C++ (must match Python name) */

    );
  }
  virtual void updateCovariance(const double lambda) VP_OVERRIDE
  {
    PYBIND11_OVERRIDE(
      void,           /* Return type */
      vpRBFeatureTracker,     /* Parent class */
      updateCovariance,        /* Name of function in C++ (must match Python name) */
      lambda
    );
  }
  virtual double getVVSTrackerWeight() const VP_OVERRIDE
  {
    PYBIND11_OVERRIDE(
      double,           /* Return type */
      vpRBFeatureTracker,     /* Parent class */
      getVVSTrackerWeight        /* Name of function in C++ (must match Python name) */
    );
  }
  virtual vpMatrix getLTL() const VP_OVERRIDE
  {
    PYBIND11_OVERRIDE(
      vpMatrix,           /* Return type */
      vpRBFeatureTracker,     /* Parent class */
      getLTL
    );
  }
  virtual vpColVector getLTR() const VP_OVERRIDE
  {
    PYBIND11_OVERRIDE(
      vpColVector,           /* Return type */
      vpRBFeatureTracker,     /* Parent class */
      getLTR        /* Name of function in C++ (must match Python name) */
    );
  }

};



#endif
