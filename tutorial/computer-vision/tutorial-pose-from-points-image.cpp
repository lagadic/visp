/*! \example tutorial-pose-from-points-image.cpp */
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/blob/vpDot2.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/vision/vpPose.h>

void computePose(std::vector<vpPoint> &point, const std::vector<vpDot2> &dot,
                 const vpCameraParameters &cam, bool init, vpHomogeneousMatrix &cMo)
{
  vpPose pose;     double x=0, y=0;
  for (unsigned int i=0; i < point.size(); i ++) {
    vpPixelMeterConversion::convertPoint(cam, dot[i].getCog(), x, y);
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

int main()
{
  try {
    vpImage<unsigned char> I;
    vpImageIo::read(I, "square.pgm");

#if defined(VISP_HAVE_X11)
    vpDisplayX d(I);
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI d(I);
#elif defined(VISP_HAVE_OPENCV)
    vpDisplayOpenCV d(I);
#endif

    vpCameraParameters cam(840, 840, I.getWidth()/2, I.getHeight()/2);
    std::vector<vpDot2> dot(4);
    dot[0].initTracking(I, vpImagePoint(193, 157));
    dot[1].initTracking(I, vpImagePoint(203, 366));
    dot[2].initTracking(I, vpImagePoint(313, 402));
    dot[3].initTracking(I, vpImagePoint(304, 133));
    std::vector<vpPoint> point;
    point.push_back( vpPoint(-0.06, -0.06, 0) );
    point.push_back( vpPoint( 0.06, -0.06, 0) );
    point.push_back( vpPoint( 0.06,  0.06, 0) );
    point.push_back( vpPoint(-0.06,  0.06, 0) );
    vpHomogeneousMatrix cMo;
    bool init = true;

    while(1) {
      vpImageIo::read(I, "square.pgm");
      vpDisplay::display(I);
      for (unsigned int i=0; i < dot.size(); i ++) {
        dot[i].setGraphics(true);
        dot[i].track(I);
      }
      computePose(point, dot, cam, init, cMo);
      vpDisplay::displayFrame(I, cMo, cam, 0.05, vpColor::none);
      vpDisplay::flush(I);

      if (init) init = false; // turn off pose initialisation

      if (vpDisplay::getClick(I, false)) break;

      vpTime::wait(40);
    }
  }
  catch(const vpException &e) {
    std::cout << "Catch an exception: " << e.getMessage() << std::endl;
  }
}
