#ifndef __pose_helper_h_
#define __pose_helper_h_

//! [Include]
#include <visp3/blob/vpDot2.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpPoint.h>
//! [Include]

void computePose(std::vector<VISP_NAMESPACE_ADDRESSING vpPoint> &point, const std::vector<VISP_NAMESPACE_ADDRESSING vpImagePoint> &ip, const VISP_NAMESPACE_ADDRESSING vpCameraParameters &cam,
                 bool init, VISP_NAMESPACE_ADDRESSING vpHomogeneousMatrix &cMo);

#if defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV)
std::vector<VISP_NAMESPACE_ADDRESSING vpImagePoint> track(VISP_NAMESPACE_ADDRESSING vpImage<unsigned char> &I, std::vector<VISP_NAMESPACE_ADDRESSING vpDot2> &dot, bool init);
#endif

#endif
