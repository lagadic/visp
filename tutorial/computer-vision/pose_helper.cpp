#include <visp3/vision/vpPose.h>
#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpPixelMeterConversion.h>

#include "pose_helper.h"

//! [Compute pose]
void computePose(std::vector<vpPoint> &point, const std::vector<vpImagePoint> &ip,
                 const vpCameraParameters &cam, bool init, vpHomogeneousMatrix &cMo)
{
  vpPose pose;
  double x = 0, y = 0;
  for (unsigned int i = 0; i < point.size(); i++) {
    vpPixelMeterConversion::convertPoint(cam, ip[i], x, y);
    point[i].set_x(x);
    point[i].set_y(y);
    pose.addPoint(point[i]);
  }

  if (init == true) {
    vpHomogeneousMatrix cMo_dem;
    vpHomogeneousMatrix cMo_lag;
    pose.computePose(vpPose::DEMENTHON, cMo_dem);
    pose.computePose(vpPose::LAGRANGE, cMo_lag);
    double residual_dem = pose.computeResidual(cMo_dem);
    double residual_lag = pose.computeResidual(cMo_lag);
    if (residual_dem < residual_lag)
      cMo = cMo_dem;
    else
      cMo = cMo_lag;
  }
  pose.computePose(vpPose::VIRTUAL_VS, cMo);
}
//! [Compute pose]

#if defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV)
std::vector<vpImagePoint> track(vpImage<unsigned char> &I, std::vector<vpDot2> &dot, bool init)
{
  try {
    double distance_same_blob = 10.; // 2 blobs are declared same if their distance is less than this value
    std::vector<vpImagePoint> ip(dot.size());
    if (init) {
      vpDisplay::flush(I);
      for (unsigned int i = 0; i < dot.size(); i++) {
        dot[i].setGraphics(true);
        dot[i].setGraphicsThickness(2);
        std::stringstream ss;
        ss << "Click on point " << i+1;
        vpDisplay::displayText(I, 20, 20, "Status: initialize blob tracker", vpColor::red);
        vpDisplay::displayText(I, 40 + i*20, 20, ss.str(), vpColor::red);
        vpDisplay::flush(I);
        dot[i].initTracking(I);
        vpDisplay::flush(I);
      }
    } else {
      for (unsigned int i = 0; i < dot.size(); i++) {
        dot[i].track(I);
      }
    }
    for (unsigned int i = 0; i < dot.size(); i++) {
      ip[i] = dot[i].getCog();
      // Compare distances between all the dots to check if some of them are not the same
    }
    for (unsigned int i=0; i < ip.size(); i++) {
      for (unsigned int j=i+1; j < ip.size(); j++) {
        if (vpImagePoint::distance(ip[i], ip[j]) < distance_same_blob) {
          std::cout << "Traking lost: 2 blobs are the same" << std::endl;
          throw(vpException(vpException::fatalError, "Tracking lost: 2 blobs are the same"));
        }
      }
    }

    return ip;
  }
  catch(...) {
    std::cout << "Traking lost" << std::endl;
    throw(vpException(vpException::fatalError, "Tracking lost"));
  }
}
#endif

