//! \example tutorial-barcode-detector-live.cpp
#include <iostream>

#include <visp3/core/vpConfig.h>

//! [Undef grabber]
// Comment / uncomment following lines to use the specific 3rd party compatible with your camera
// #undef VISP_HAVE_V4L2
// #undef HAVE_OPENCV_HIGHGUI
// #undef HAVE_OPENCV_VIDEOIO
//! [Undef grabber]

#if (defined(VISP_HAVE_ZBAR) || defined(VISP_HAVE_DMTX)) && (defined(VISP_HAVE_V4L2) || \
     ((VISP_HAVE_OPENCV_VERSION < 0x030000) && defined(HAVE_OPENCV_HIGHGUI))|| ((VISP_HAVE_OPENCV_VERSION >= 0x030000) && defined(HAVE_OPENCV_VIDEOIO)))

#include <visp3/core/vpImageConvert.h>
#include <visp3/detection/vpDetectorDataMatrixCode.h>
#include <visp3/detection/vpDetectorQRCode.h>
#include <visp3/gui/vpDisplayFactory.h>
#ifdef VISP_HAVE_MODULE_SENSOR
#include <visp3/sensor/vpV4l2Grabber.h>
#endif

#if (VISP_HAVE_OPENCV_VERSION < 0x030000) && defined(HAVE_OPENCV_HIGHGUI)
#include <opencv2/highgui/highgui.hpp> // for cv::VideoCapture
#elif (VISP_HAVE_OPENCV_VERSION >= 0x030000) && defined(HAVE_OPENCV_VIDEOIO)
#include <opencv2/videoio/videoio.hpp> // for cv::VideoCapture
#endif

int main(int argc, const char **argv)
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif
  int opt_device = 0;
  int opt_barcode = 0; // 0=QRCode, 1=DataMatrix

  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--device" && i + 1 < argc) {
      opt_device = atoi(argv[++i]);
    }
    else if (std::string(argv[i]) == "--code-type" && i + 1 < argc) {
      opt_barcode = atoi(argv[++i]);
    }
    else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      std::cout << "Usage: " << argv[0]
        << " [--device <camera number>]"
        << " [--code-type <0 for QR code | 1 for DataMatrix code>]"
        << " [--help] [-h]"
        << std::endl;
      return EXIT_SUCCESS;
    }
  }
  std::cout << "Use device: " << opt_device << std::endl;

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  std::shared_ptr<vpDisplay> display;
#else
  vpDisplay *display = nullptr;
#endif
  try {
    vpImage<unsigned char> I; // for gray images

    //! [Construct grabber]
#if defined(VISP_HAVE_V4L2)
    vpV4l2Grabber g;
    std::ostringstream device;
    device << "/dev/video" << opt_device;
    g.setDevice(device.str());
    g.setScale(1);
    g.acquire(I);
#elif ((VISP_HAVE_OPENCV_VERSION < 0x030000) && defined(HAVE_OPENCV_HIGHGUI))|| ((VISP_HAVE_OPENCV_VERSION >= 0x030000) && defined(HAVE_OPENCV_VIDEOIO))
    cv::VideoCapture cap(opt_device); // open the default camera
    if (!cap.isOpened()) {            // check if we succeeded
      std::cout << "Failed to open the camera" << std::endl;
      return EXIT_FAILURE;
    }
    cv::Mat frame;
    cap >> frame; // get a new frame from camera
    vpImageConvert::convert(frame, I);
#endif
    //! [Construct grabber]

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
    display = vpDisplayFactory::createDisplay(I);
#else
    display = vpDisplayFactory::allocateDisplay(I);
#endif
    vpDisplay::setTitle(I, "ViSP viewer");

    vpDetectorBase *detector = nullptr;
#if (defined(VISP_HAVE_ZBAR) && defined(VISP_HAVE_DMTX))
    if (opt_barcode == 0)
      detector = new vpDetectorQRCode;
    else
      detector = new vpDetectorDataMatrixCode;
#elif defined(VISP_HAVE_ZBAR)
    detector = new vpDetectorQRCode;
    (void)opt_barcode;
#elif defined(VISP_HAVE_DMTX)
    detector = new vpDetectorDataMatrixCode;
    (void)opt_barcode;
#endif

    for (;;) {
      //! [Acquisition]
#if defined(VISP_HAVE_V4L2)
      g.acquire(I);
#elif ((VISP_HAVE_OPENCV_VERSION < 0x030000) && defined(HAVE_OPENCV_HIGHGUI))|| ((VISP_HAVE_OPENCV_VERSION >= 0x030000) && defined(HAVE_OPENCV_VIDEOIO))
      cap >> frame; // get a new frame from camera
      vpImageConvert::convert(frame, I);
#endif
      //! [Acquisition]
      vpDisplay::display(I);

      bool status = detector->detect(I);
      std::ostringstream legend;
      legend << detector->getNbObjects() << " bar code detected";
      vpDisplay::displayText(I, 10, 10, legend.str(), vpColor::red);

      if (status) {
        for (size_t i = 0; i < detector->getNbObjects(); i++) {
          std::vector<vpImagePoint> p = detector->getPolygon(i);
          vpRect bbox = detector->getBBox(i);
          vpDisplay::displayRectangle(I, bbox, vpColor::green);
          vpDisplay::displayText(I, static_cast<int>(bbox.getTop()) - 20, static_cast<int>(bbox.getLeft()),
                                 "Message: \"" + detector->getMessage(i) + "\"", vpColor::red);
          for (size_t j = 0; j < p.size(); j++) {
            vpDisplay::displayCross(I, p[j], 14, vpColor::red, 3);
            std::ostringstream number;
            number << j;
            vpDisplay::displayText(I, p[j] + vpImagePoint(10, 0), number.str(), vpColor::blue);
          }
        }
      }

      vpDisplay::displayText(I, static_cast<int>(I.getHeight()) - 25, 10, "Click to quit...", vpColor::red);
      vpDisplay::flush(I);
      if (vpDisplay::getClick(I, false)) // a click to exit
        break;
    }
    delete detector;
  }
  catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }
#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
  if (display != nullptr) {
    delete display;
  }
#endif
}

#else

int main()
{
  std::cout << "There are missing 3rd parties to run this tutorial" << std::endl;
}

#endif
