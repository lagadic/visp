//! \example tutorial-blob-tracker-live-firewire.cpp
#include <visp/vp1394CMUGrabber.h>
#include <visp/vp1394TwoGrabber.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDot2.h>

int main()
{
#if (defined(VISP_HAVE_DC1394_2) || defined(VISP_HAVE_CMU1394) || (VISP_HAVE_OPENCV_VERSION >= 0x020100)) && (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV))
  vpImage<unsigned char> I; // Create a gray level image container

#if defined(VISP_HAVE_DC1394_2)
  vp1394TwoGrabber g(false);
  g.open(I);
#elif defined(VISP_HAVE_CMU1394)
  vp1394CMUGrabber g;
  g.open(I);
#elif defined(VISP_HAVE_OPENCV)
  cv::VideoCapture g(0); // open the default camera
  if(!g.isOpened()) { // check if we succeeded
    std::cout << "Failed to open the camera" << std::endl;
    return -1;
  }
  cv::Mat frame;
  g >> frame; // get a new frame from camera
  vpImageConvert::convert(frame, I);
#endif

#if defined(VISP_HAVE_X11)
  vpDisplayX d(I, 0, 0, "Camera view");
#elif defined(VISP_HAVE_GDI)
  vpDisplayGDI d(I, 0, 0, "Camera view");
#elif defined(VISP_HAVE_OPENCV)
  vpDisplayOpenCV d(I, 0, 0, "Camera view");
#endif

  //! [Construction]
  vpDot2 blob;
  //! [Construction]
  //! [Setting]
  blob.setGraphics(true);
  blob.setGraphicsThickness(2);
  //! [Setting]

  vpImagePoint germ;
  bool init_done = false;

  while(1) {
    try {
#if defined(VISP_HAVE_DC1394_2) || defined(VISP_HAVE_CMU1394)
      g.acquire(I);
#elif defined(VISP_HAVE_OPENCV)
      g >> frame;
      vpImageConvert::convert(frame, I);
#endif
      vpDisplay::display(I);

      if (! init_done) {
        vpDisplay::displayText(I, vpImagePoint(10,10), "Click in the blob to initialize the tracker", vpColor::red);
        if (vpDisplay::getClick(I, germ, false)) {
          //! [Init]
          blob.initTracking(I, germ);
          //! [Init]
          init_done = true;
        }
      }
      else {
        //! [Track]
        blob.track(I);
        //! [Track]
      }

      vpDisplay::flush(I);
    }
    catch(...) {
      init_done = false;
    }
  }
#endif
}
