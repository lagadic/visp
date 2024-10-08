#ifndef VP_RB_VISUAL_ODOMETRY_H
#define VP_RB_VISUAL_ODOMETRY_H

#include <visp3/core/vpConfig.h>

class vpRBFeatureTrackerInput;
class vpHomogeneousMatrix;

class VISP_EXPORT vpRBVisualOdometry
{
public:
  virtual void compute(const vpRBFeatureTrackerInput &frame, const vpRBFeatureTrackerInput &previousFrame) = 0;
  virtual vpHomogeneousMatrix getCameraMotion() const = 0;
  virtual vpHomogeneousMatrix getCameraPose() const = 0;
};


#endif
