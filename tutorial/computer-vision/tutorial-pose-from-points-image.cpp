/*! \example tutorial-pose-from-points-image.cpp */
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDot2.h>
#include <visp/vpImageIo.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpPose.h>

void computePose(std::vector<vpPoint> &point, const std::vector<vpDot2> &dot,
                 const vpCameraParameters &cam, bool init, vpHomogeneousMatrix &cMo);

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

  if (init == true) pose.computePose(vpPose::DEMENTHON_VIRTUAL_VS, cMo);
  else              pose.computePose(vpPose::VIRTUAL_VS, cMo) ;
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
#endif

    vpCameraParameters cam(840, 840, I.getWidth()/2, I.getHeight()/2);
    std::vector<vpDot2> dot(4);
    dot[0].initTracking(I, vpImagePoint(193, 157));
    dot[1].initTracking(I, vpImagePoint(203, 366));
    dot[2].initTracking(I, vpImagePoint(313, 402));
    dot[3].initTracking(I, vpImagePoint(304, 133));
    std::vector<vpPoint> point(4);
    point[0].setWorldCoordinates(-0.06, -0.06, 0);
    point[1].setWorldCoordinates( 0.06, -0.06, 0);
    point[2].setWorldCoordinates( 0.06,  0.06, 0);
    point[3].setWorldCoordinates(-0.06,  0.06, 0);
    vpHomogeneousMatrix cMo;
    bool init = true;

    while(1){
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
  catch(vpException e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }
}
