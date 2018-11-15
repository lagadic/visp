#ifndef __pose_helper_h_
#define __pose_helper_h_

#include <visp3/core/vpPoint.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpImage.h>
#include <visp3/blob/vpDot2.h>

void computePose(std::vector<vpPoint> &point, const std::vector<vpImagePoint> &ip,
                 const vpCameraParameters &cam, bool init, vpHomogeneousMatrix &cMo);

void computePose(std::vector<vpPoint> &point, const std::vector<vpDot2> &dot,
                 const vpCameraParameters &cam, bool init, vpHomogeneousMatrix &cMo);

#if defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV)
void track(vpImage<unsigned char> &I, std::vector<vpDot2> &dot, bool init);
#endif

#endif

