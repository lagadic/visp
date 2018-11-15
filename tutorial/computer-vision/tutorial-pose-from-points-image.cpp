/*! \example tutorial-pose-from-points-image.cpp */
#include <visp3/blob/vpDot2.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/vision/vpPose.h>

#include "pose_helper.h"

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

    vpCameraParameters cam(840, 840, I.getWidth() / 2, I.getHeight() / 2);
    std::vector<vpDot2> dot(4);
    dot[0].initTracking(I, vpImagePoint(193, 157));
    dot[1].initTracking(I, vpImagePoint(203, 366));
    dot[2].initTracking(I, vpImagePoint(313, 402));
    dot[3].initTracking(I, vpImagePoint(304, 133));
    std::vector<vpPoint> point;
    point.push_back(vpPoint(-0.06, -0.06, 0));
    point.push_back(vpPoint(0.06, -0.06, 0));
    point.push_back(vpPoint(0.06, 0.06, 0));
    point.push_back(vpPoint(-0.06, 0.06, 0));
    vpHomogeneousMatrix cMo;
    bool init = true;

    while (1) {
      vpImageIo::read(I, "square.pgm");
      vpDisplay::display(I);
      for (unsigned int i = 0; i < dot.size(); i++) {
        dot[i].setGraphics(true);
        dot[i].track(I);
      }
      computePose(point, dot, cam, init, cMo);
      vpDisplay::displayFrame(I, cMo, cam, 0.05, vpColor::none);
      vpDisplay::flush(I);

      if (init)
        init = false; // turn off pose initialisation

      if (vpDisplay::getClick(I, false))
        break;

      vpTime::wait(40);
    }
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e.getMessage() << std::endl;
  }
}
