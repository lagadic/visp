//! \example tutorial-blob-tracker-live-v4l2.cpp
#include <visp3/core/vpConfig.h>
#ifdef VISP_HAVE_MODULE_SENSOR
#include <visp3/sensor/vpV4l2Grabber.h>
#endif
#include <visp3/blob/vpDot2.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>

#if defined(HAVE_OPENCV_VIDEOIO)
#include <opencv2/videoio.hpp>
#endif

int main(int argc, char **argv)
{
#if ((defined(VISP_HAVE_V4L2) || defined(HAVE_OPENCV_VIDEOIO)) &&                                            \
     (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV) || defined(VISP_HAVE_GTK)))

#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif

  int device = 0;
  if (argc == 2) {
    device = std::atoi(argv[1]);
  }
  vpImage<unsigned char> I; // Create a gray level image container

  std::stringstream ss;
  ss << "/dev/video" << device;

  std::cout << "Connect to: " << ss.str() << std::endl;
#if defined(VISP_HAVE_V4L2)
  vpV4l2Grabber g;
  g.setDevice(ss.str());
  g.open(I);
#elif defined(HAVE_OPENCV_VIDEOIO)
  cv::VideoCapture g(device); // open the default camera
  if (!g.isOpened()) {        // check if we succeeded
    std::cout << "Failed to open the camera" << std::endl;
    return EXIT_FAILURE;
  }
  cv::Mat frame;
  g >> frame; // get a new frame from camera
  vpImageConvert::convert(frame, I);
#endif

#if defined(VISP_HAVE_X11)
  vpDisplayX d(I, 0, 0, "Camera view");
#elif defined(VISP_HAVE_GDI)
  vpDisplayGDI d(I, 0, 0, "Camera view");
#elif defined(HAVE_OPENCV_HIGHGUI)
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
  while (1) {
    try {
#if defined(VISP_HAVE_V4L2)
      g.acquire(I);
#elif defined(HAVE_OPENCV_VIDEOIO)
      g >> frame;
      vpImageConvert::convert(frame, I);
#endif
      vpDisplay::display(I);

      if (!init_done) {
        vpDisplay::displayText(I, vpImagePoint(10, 10), "Click in the blob to initialize the tracker", vpColor::red);
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
    catch (...) {
      init_done = false;
    }
  }
#else
  (void)argc;
  (void)argv;
#endif
}
