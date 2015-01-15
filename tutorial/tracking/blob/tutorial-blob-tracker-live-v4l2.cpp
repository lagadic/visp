//! \example tutorial-blob-tracker-live-v4l2.cpp
#include <visp/vpV4l2Grabber.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayGTK.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDot2.h>

int main()
{
#if ((defined(VISP_HAVE_V4L2) || (VISP_HAVE_OPENCV_VERSION >= 0x020100)) && (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV) || defined(VISP_HAVE_GTK)))
  vpImage<unsigned char> I; // Create a gray level image container

#if defined(VISP_HAVE_V4L2)
  vpV4l2Grabber g;
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
#elif defined(VISP_HAVE_GTK)
  vpDisplayGTK d(I, 0, 0, "Camera view");
#endif

  vpDot2 blob;
  blob.setGraphics(true);
  blob.setGraphicsThickness(2);

  vpImagePoint germ;
  bool init_done = false;
  std::cout << "Click!!!" << std::endl;
  while(1) {
    try {
#if defined(VISP_HAVE_V4L2)
      g.acquire(I);
#elif defined(VISP_HAVE_OPENCV)
      g >> frame;
      vpImageConvert::convert(frame, I);
#endif
      vpDisplay::display(I);

      if (! init_done) {
        vpDisplay::displayText(I, vpImagePoint(10,10), "Click in the blob to initialize the tracker", vpColor::red);
        if (vpDisplay::getClick(I, germ, false)) {
          blob.initTracking(I, germ);
          init_done = true;
        }
      }
      else {
        blob.track(I);
      }
      vpDisplay::flush(I);
    }
    catch(...) {
      init_done = false;
    }
  }
#endif
}
