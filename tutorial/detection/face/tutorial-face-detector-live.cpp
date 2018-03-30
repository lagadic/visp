//! \example tutorial-face-detector-live.cpp
#include <visp3/core/vpConfig.h>
#include <visp3/detection/vpDetectorFace.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#ifdef VISP_HAVE_MODULE_SENSOR
#include <visp3/sensor/vpV4l2Grabber.h>
#endif

int main(int argc, const char *argv[])
{
#if (VISP_HAVE_OPENCV_VERSION >= 0x020200)
  try {
    std::string opt_face_cascade_name = "./haarcascade_frontalface_alt.xml";
    unsigned int opt_device = 0;
    unsigned int opt_scale = 2; // Default value is 2 in the constructor. Turn
                                // it to 1 to avoid subsampling

    for (int i = 0; i < argc; i++) {
      if (std::string(argv[i]) == "--haar")
        opt_face_cascade_name = std::string(argv[i + 1]);
      else if (std::string(argv[i]) == "--device")
        opt_device = (unsigned int)atoi(argv[i + 1]);
      else if (std::string(argv[i]) == "--scale")
        opt_scale = (unsigned int)atoi(argv[i + 1]);
      else if (std::string(argv[i]) == "--help") {
        std::cout << "Usage: " << argv[0]
                  << " [--haar <haarcascade xml filename>] [--device <camera "
                     "device>] [--scale <subsampling factor>] [--help]"
                  << std::endl;
        return 0;
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
#elif defined(VISP_HAVE_OPENCV)
    cv::VideoCapture cap(opt_device); // open the default camera
#if (VISP_HAVE_OPENCV_VERSION >= 0x030000)
    int width = (int)cap.get(cv::CAP_PROP_FRAME_WIDTH);
    int height = (int)cap.get(cv::CAP_PROP_FRAME_HEIGHT);
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
      return -1;
    }
    cv::Mat frame;
    cap >> frame; // get a new frame from camera
    vpImageConvert::convert(frame, I);
#endif
//! [Construct grabber]

#if defined(VISP_HAVE_X11)
    vpDisplayX d(I);
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI d(I);
#elif defined(VISP_HAVE_OPENCV)
    vpDisplayOpenCV d(I);
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
#else
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
          vpDisplay::displayText(I, (int)bbox.getTop() - 10, (int)bbox.getLeft(),
                                 "Message: \"" + face_detector.getMessage(i) + "\"", vpColor::red);
        }
      }
      vpDisplay::displayText(I, (int)I.getHeight() - 25, 10, "Click to quit...", vpColor::red);
      vpDisplay::flush(I);
      if (vpDisplay::getClick(I, false)) // a click to exit
        break;

      std::cout << "Loop time: " << vpTime::measureTimeMs() - t << " ms" << std::endl;
    }
  } catch (const vpException &e) {
    std::cout << e.getMessage() << std::endl;
  }
#else
  (void)argc;
  (void)argv;
#endif
}
