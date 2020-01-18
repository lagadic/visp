/*! \example tutorial-pose-from-points-image.cpp */
#include <visp3/io/vpImageIo.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>

//! [Include]
#include "pose_helper.h"
//! [Include]

int main()
{
  try {
    //! [Read image]
    vpImage<unsigned char> I;
    vpImageIo::read(I, "square.pgm");
    //! [Read image]

#if defined(VISP_HAVE_X11)
    vpDisplayX d(I);
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI d(I);
#elif defined(VISP_HAVE_OPENCV)
    vpDisplayOpenCV d(I);
#endif

    //! [Camera parameters]
    vpCameraParameters cam(840, 840, I.getWidth() / 2, I.getHeight() / 2);
    //! [Camera parameters]
    //! [Blob tracking]
    std::vector<vpDot2> dot(4);
    std::vector<vpImagePoint> ip(4);
    dot[0].initTracking(I, vpImagePoint(193, 157));
    dot[1].initTracking(I, vpImagePoint(203, 366));
    dot[2].initTracking(I, vpImagePoint(313, 402));
    dot[3].initTracking(I, vpImagePoint(304, 133));
    //! [Blob tracking]
    //! [3D model]
    std::vector<vpPoint> point;
    point.push_back(vpPoint(-0.06, -0.06, 0));
    point.push_back(vpPoint( 0.06, -0.06, 0));
    point.push_back(vpPoint( 0.06,  0.06, 0));
    point.push_back(vpPoint(-0.06,  0.06, 0));
    //! [3D model]

    //! [Homogeneous matrix]
    vpHomogeneousMatrix cMo;
    //! [Homogeneous matrix]
    bool init = true;

    while (1) {
      //! [Tracking]
      vpImageIo::read(I, "square.pgm");
      vpDisplay::display(I);
      for (unsigned int i = 0; i < dot.size(); i++) {
        dot[i].setGraphics(true);
        dot[i].track(I);
        ip[i] = dot[i].getCog();
      }
      //! [Tracking]
      //! [Pose estimation]
      computePose(point, ip, cam, init, cMo);
      //! [Pose estimation]
      //! [Display pose]
      vpDisplay::displayFrame(I, cMo, cam, 0.05, vpColor::none);
      vpDisplay::flush(I);
      //! [Display pose]

      //! [Next pose uses previous one as initialization]
      if (init)
        init = false; // turn off pose initialisation
      //! [Next pose uses previous one as initialization]

      //! [The end]
      if (vpDisplay::getClick(I, false))
        break;
      //! [The end]

      //! [Slow down]
      vpTime::wait(40);
      //! [Slow down]
    }
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e.getMessage() << std::endl;
  }
}
