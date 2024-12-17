//! \example tutorial-blob-tracker-live.cpp
#include <iostream>

#include <visp3/core/vpConfig.h>

//! [Undef grabber]
// Comment / uncomment following lines to use the specific 3rd party compatible with your camera
// #undef VISP_HAVE_V4L2
// #undef VISP_HAVE_DC1394
// #undef VISP_HAVE_CMU1394
// #undef VISP_HAVE_FLYCAPTURE
// #undef VISP_HAVE_REALSENSE2
// #undef HAVE_OPENCV_HIGHGUI
// #undef HAVE_OPENCV_VIDEOIO
//! [Undef grabber]

#if (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV)) && \
    (defined(VISP_HAVE_V4L2) || defined(VISP_HAVE_DC1394) || defined(VISP_HAVE_CMU1394) || \
     defined(VISP_HAVE_FLYCAPTURE) || defined(VISP_HAVE_REALSENSE2) || \
     ((VISP_HAVE_OPENCV_VERSION < 0x030000) && defined(HAVE_OPENCV_HIGHGUI)) || ((VISP_HAVE_OPENCV_VERSION >= 0x030000) && defined(HAVE_OPENCV_VIDEOIO)))

#ifdef VISP_HAVE_MODULE_SENSOR
#include <visp3/sensor/vp1394CMUGrabber.h>
#include <visp3/sensor/vp1394TwoGrabber.h>
#include <visp3/sensor/vpFlyCaptureGrabber.h>
#include <visp3/sensor/vpRealSense2.h>
#include <visp3/sensor/vpV4l2Grabber.h>
#endif
#include <visp3/blob/vpDot2.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>

#if (VISP_HAVE_OPENCV_VERSION < 0x030000) && defined(HAVE_OPENCV_HIGHGUI)
#include <opencv2/highgui/highgui.hpp> // for cv::VideoCapture
#elif (VISP_HAVE_OPENCV_VERSION >= 0x030000) && defined(HAVE_OPENCV_VIDEOIO)
#include <opencv2/videoio/videoio.hpp>
#endif

int main()
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif

  vpImage<unsigned char> I; // Create a gray level image container
  int opt_device = 0; // For OpenCV and V4l2 grabber to set the camera device

    //! [Grabber]
#if defined(VISP_HAVE_V4L2)
  vpV4l2Grabber g;
  std::ostringstream device;
  device << "/dev/video" << opt_device;
  std::cout << "Use Video 4 Linux grabber on device " << device.str() << std::endl;
  g.setDevice(device.str());
  g.setScale(1);
  g.open(I);
#elif defined(VISP_HAVE_DC1394)
  (void)opt_device; // To avoid non used warning
  std::cout << "Use DC1394 grabber" << std::endl;
  vp1394TwoGrabber g;
  g.open(I);
#elif defined(VISP_HAVE_CMU1394)
  (void)opt_device; // To avoid non used warning
  std::cout << "Use CMU1394 grabber" << std::endl;
  vp1394CMUGrabber g;
  g.open(I);
#elif defined(VISP_HAVE_FLYCAPTURE)
  (void)opt_device; // To avoid non used warning
  std::cout << "Use FlyCapture grabber" << std::endl;
  vpFlyCaptureGrabber g;
  g.open(I);
#elif defined(VISP_HAVE_REALSENSE2)
  (void)opt_device; // To avoid non used warning
  std::cout << "Use Realsense 2 grabber" << std::endl;
  vpRealSense2 g;
  rs2::config config;
  config.disable_stream(RS2_STREAM_DEPTH);
  config.disable_stream(RS2_STREAM_INFRARED);
  config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGBA8, 30);
  g.open(config);
  g.acquire(I);

  std::cout << "Read camera parameters from Realsense device" << std::endl;
  cam = g.getCameraParameters(RS2_STREAM_COLOR, vpCameraParameters::perspectiveProjWithoutDistortion);
#elif ((VISP_HAVE_OPENCV_VERSION < 0x030000) && defined(HAVE_OPENCV_HIGHGUI))|| ((VISP_HAVE_OPENCV_VERSION >= 0x030000) && defined(HAVE_OPENCV_VIDEOIO))
  cv::VideoCapture g(opt_device); // open the default camera
  if (!g.isOpened()) {   // check if we succeeded
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
  bool quit = false;
  bool germ_selected = false;
  vpMouseButton::vpMouseButtonType button;

  while (! quit) {
    try {
#if defined(VISP_HAVE_V4L2) || defined(VISP_HAVE_DC1394) || defined(VISP_HAVE_CMU1394) || defined(VISP_HAVE_FLYCAPTURE) || defined(VISP_HAVE_REALSENSE2)
      g.acquire(I);
#elif ((VISP_HAVE_OPENCV_VERSION < 0x030000) && defined(HAVE_OPENCV_HIGHGUI))|| ((VISP_HAVE_OPENCV_VERSION >= 0x030000) && defined(HAVE_OPENCV_VIDEOIO))
      g >> frame;
      vpImageConvert::convert(frame, I);
#endif
      vpDisplay::display(I);
      vpDisplay::displayText(I, 20, 20, "Left click in the blob to initialize the tracker", vpColor::red);
      vpDisplay::displayText(I, 40, 20, "Right click to quit", vpColor::red);

      if (vpDisplay::getClick(I, germ, button, false)) {
        if (button == vpMouseButton::button3) {
          quit = true;
        }
        else {
          germ_selected = true;
        }
      }
      if (germ_selected && !init_done) {
        //! [Init]
        std::cout << "Tracking initialized" << std::endl;
        blob.initTracking(I, germ);
        //! [Init]
        init_done = true;
        germ_selected = false;
      }
      else if (init_done) {
        //! [Track]
        blob.track(I);
        //! [Track]
      }

      vpDisplay::flush(I);
    }
    catch (const vpException &e) {
      std::cout << "Tracking failed: " << e.getMessage() << std::endl;
      init_done = false;
    }
  }
}

#else
int main()
{
  std::cout << "There are missing 3rd parties to run this tutorial" << std::endl;
}
#endif
