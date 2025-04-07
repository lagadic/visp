//! \example tutorial-face-detector-live.cpp
#include <iostream>

#include <visp3/core/vpConfig.h>

//! [Undef grabber]
// Comment / uncomment following lines to use the specific 3rd party compatible with your camera
// #undef VISP_HAVE_V4L2
// #undef HAVE_OPENCV_HIGHGUI
// #undef HAVE_OPENCV_VIDEOIO
//! [Undef grabber]

#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGPROC) && \
    (((VISP_HAVE_OPENCV_VERSION < 0x050000) && defined(HAVE_OPENCV_OBJDETECT)) || \
     ((VISP_HAVE_OPENCV_VERSION >= 0x050000) && defined(HAVE_OPENCV_XOBJDETECT))) && \
    (defined(VISP_HAVE_V4L2) || \
    (((VISP_HAVE_OPENCV_VERSION < 0x030000) && defined(HAVE_OPENCV_HIGHGUI)) || \
     ((VISP_HAVE_OPENCV_VERSION >= 0x030000) && defined(HAVE_OPENCV_VIDEOIO))))

#include <visp3/detection/vpDetectorFace.h>
#include <visp3/gui/vpDisplayFactory.h>
#ifdef VISP_HAVE_MODULE_SENSOR
#include <visp3/sensor/vpV4l2Grabber.h>
#endif

#if (VISP_HAVE_OPENCV_VERSION < 0x030000) && defined(HAVE_OPENCV_HIGHGUI)
#include <opencv2/highgui/highgui.hpp> // for cv::VideoCapture
#elif (VISP_HAVE_OPENCV_VERSION >= 0x030000) && defined(HAVE_OPENCV_VIDEOIO)
#include <opencv2/videoio/videoio.hpp>
#endif

int main(int argc, const char *argv[])
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  std::shared_ptr<vpDisplay> display;
#else
  vpDisplay *display = nullptr;
#endif
  try {
    std::string opt_face_cascade_name = "./haarcascade_frontalface_alt.xml";
    unsigned int opt_device = 0;
    unsigned int opt_scale = 2; // Default value is 2 in the constructor. Turn
    // it to 1 to avoid subsampling

    for (int i = 1; i < argc; i++) {
      if (std::string(argv[i]) == "--haar" && i + 1 < argc) {
        opt_face_cascade_name = std::string(argv[++i]);
      }
      else if (std::string(argv[i]) == "--device" && i + 1 < argc) {
        opt_device = static_cast<unsigned int>(atoi(argv[++i]));
      }
      else if (std::string(argv[i]) == "--scale" && i + 1 < argc) {
        opt_scale = static_cast<unsigned int>(atoi(argv[++i]));
      }
      else if ((std::string(argv[i]) == "--help") || (std::string(argv[i]) == "-h")) {
        std::cout << "Usage: " << argv[0]
          << " [--haar <haarcascade xml filename>]"
          << " [--device <camera device>]"
          << " [--scale <subsampling factor>]"
          << " [--help] [-h]"
          << std::endl;
        return EXIT_SUCCESS;
      }
    }

    vpImage<unsigned char> I; // for gray images

    //! [Construct grabber]
#if defined(VISP_HAVE_V4L2)
    vpV4l2Grabber g;
    std::ostringstream device;
    device << "/dev/video" << opt_device;
    g.setDevice(device.str());
    g.setScale(opt_scale); // Default value is 2 in the constructor. Turn it
    // to 1 to avoid subsampling
    g.acquire(I);
#elif ((VISP_HAVE_OPENCV_VERSION < 0x030000) && defined(HAVE_OPENCV_HIGHGUI))|| ((VISP_HAVE_OPENCV_VERSION >= 0x030000) && defined(HAVE_OPENCV_VIDEOIO))
    cv::VideoCapture cap(opt_device); // open the default camera
#if (VISP_HAVE_OPENCV_VERSION >= 0x030000)
    int width = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
    int height = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
    cap.set(cv::CAP_PROP_FRAME_WIDTH, width / opt_scale);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, height / opt_scale);
#else
    int width = cap.get(CV_CAP_PROP_FRAME_WIDTH);
    int height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
    cap.set(CV_CAP_PROP_FRAME_WIDTH, width / opt_scale);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, height / opt_scale);
#endif
    if (!cap.isOpened()) { // check if we succeeded
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

    vpDetectorFace face_detector;
    face_detector.setCascadeClassifierFile(opt_face_cascade_name);

    while (1) {
      double t = vpTime::measureTimeMs();
      //! [Acquisition]
#if defined(VISP_HAVE_V4L2)
      g.acquire(I);
      bool face_found = face_detector.detect(I);
#elif ((VISP_HAVE_OPENCV_VERSION < 0x030000) && defined(HAVE_OPENCV_HIGHGUI))|| ((VISP_HAVE_OPENCV_VERSION >= 0x030000) && defined(HAVE_OPENCV_VIDEOIO))
      cap >> frame; // get a new frame from camera
      vpImageConvert::convert(frame, I);
      bool face_found = face_detector.detect(frame); // We pass frame to avoid an internal image conversion
#endif
      //! [Acquisition]

      vpDisplay::display(I);

      if (face_found) {
        std::ostringstream text;
        text << "Found " << face_detector.getNbObjects() << " face(s)";
        vpDisplay::displayText(I, 10, 10, text.str(), vpColor::red);
        for (size_t i = 0; i < face_detector.getNbObjects(); i++) {
          vpRect bbox = face_detector.getBBox(i);
          vpDisplay::displayRectangle(I, bbox, vpColor::green, false, 4);
          vpDisplay::displayText(I, static_cast<int>(bbox.getTop()) - 10, static_cast<int>(bbox.getLeft()),
                                 "Message: \"" + face_detector.getMessage(i) + "\"", vpColor::red);
        }
      }
      vpDisplay::displayText(I, static_cast<int>(I.getHeight()) - 25, 10, "Click to quit...", vpColor::red);
      vpDisplay::flush(I);
      if (vpDisplay::getClick(I, false)) // a click to exit
        break;

      std::cout << "Loop time: " << vpTime::measureTimeMs() - t << " ms" << std::endl;
    }
  }
  catch (const vpException &e) {
    std::cout << e.getMessage() << std::endl;
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
#if !defined(HAVE_OPENCV_IMGPROC)
  std::cout << "This tutorial needs OpenCV imgproc module that is missing." << std::endl;
#endif
#if defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION < 0x050000) && !defined(HAVE_OPENCV_OBJDETECT)
  std::cout << "This tutorial needs OpenCV objdetect module that is missing." << std::endl;
#endif
#if defined(VISP_HAVE_OPENCV) && ((VISP_HAVE_OPENCV_VERSION >= 0x050000) && !defined(HAVE_OPENCV_XOBJDETECT))
  std::cout << "This tutorial needs OpenCV xobjdetect module that is missing." << std::endl;
#endif
}

#endif
