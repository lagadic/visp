/*! \example tutorial-pose-from-points-tracking.cpp */
#include <visp/vp1394CMUGrabber.h>
#include <visp/vp1394TwoGrabber.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDot2.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpPose.h>

void computePose(std::vector<vpPoint> &point, const std::vector<vpDot2> &dot,
                 const vpCameraParameters &cam, bool init, vpHomogeneousMatrix &cMo);
#if defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI)
void track(vpImage<unsigned char> &I, std::vector<vpDot2> &dot, bool init);
#endif

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

#if defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI)
void track(vpImage<unsigned char> &I, std::vector<vpDot2> &dot, bool init)
{
  if (init) {
    vpDisplay::flush(I);
    for(unsigned int i=0; i<dot.size(); i++) {
      dot[i].setGraphics(true);
      dot[i].setGraphicsThickness(2);
      dot[i].initTracking(I);
      vpDisplay::flush(I);
    }
  }
  else {
    for(unsigned int i=0; i<dot.size(); i++) {
      dot[i].track(I);
    }
  }
}
#endif

int main()
{
#if (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI)) && (defined(VISP_HAVE_DC1394_2) || defined(VISP_HAVE_CMU1394))
  try {  vpImage<unsigned char> I;

#if defined(VISP_HAVE_DC1394_2)
    vp1394TwoGrabber g;
#elif defined(VISP_HAVE_CMU1394)
    vp1394CMUGrabber g;
#endif
    g.open(I);

    // Parameters of our camera
    vpCameraParameters cam(840, 840, I.getWidth()/2, I.getHeight()/2);

    // The pose container
    vpHomogeneousMatrix cMo;

    std::vector<vpDot2> dot(4);
    std::vector<vpPoint> point(4);
    double L = 0.06;
    point[0].setWorldCoordinates(-L, -L, 0);
    point[1].setWorldCoordinates( L, -L, 0);
    point[2].setWorldCoordinates( L,  L, 0);
    point[3].setWorldCoordinates(-L,  L, 0);

    bool init = true;
#if defined(VISP_HAVE_X11)
    vpDisplayX d(I);
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI d(I);
#endif

    while(1){
      // Image Acquisition
      g.acquire(I);
      vpDisplay::display(I);
      track(I, dot, init);
      computePose(point, dot, cam, init, cMo);
      vpDisplay::displayFrame(I, cMo, cam, 0.05, vpColor::none, 3);
      vpDisplay::flush(I);
      if (init) init = false; // turn off the initialisation specific stuff

      if (vpDisplay::getClick(I, false))
        break;
    }
  }
  catch(vpException e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }
#endif
}
