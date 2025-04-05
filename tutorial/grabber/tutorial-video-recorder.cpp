/*! \example tutorial-video-recorder.cpp */
#include <iostream>

#include <visp3/core/vpConfig.h>

//! [Undef grabber]
// Comment / uncomment following lines to use the specific 3rd party compatible with your camera
#undef VISP_HAVE_V4L2
// #undef HAVE_OPENCV_HIGHGUI
// #undef HAVE_OPENCV_VIDEOIO
//! [Undef grabber]

#if defined(VISP_HAVE_DISPLAY) && \
  (defined(VISP_HAVE_V4L2) || defined(VISP_HAVE_OPENCV) && \
  (((VISP_HAVE_OPENCV_VERSION < 0x030000) && defined(HAVE_OPENCV_HIGHGUI)) || ((VISP_HAVE_OPENCV_VERSION >= 0x030000) && defined(HAVE_OPENCV_VIDEOIO))))

#include <visp3/core/vpTime.h>
#include <visp3/gui/vpDisplayFactory.h>
#include <visp3/io/vpVideoWriter.h>
#include <visp3/sensor/vpV4l2Grabber.h>

#if defined(VISP_HAVE_OPENCV) &&(VISP_HAVE_OPENCV_VERSION < 0x030000) && defined(HAVE_OPENCV_HIGHGUI)
#include <opencv2/highgui/highgui.hpp> // for cv::VideoCapture
#elif defined(VISP_HAVE_OPENCV) &&(VISP_HAVE_OPENCV_VERSION >= 0x030000) && defined(HAVE_OPENCV_VIDEOIO)
#include <opencv2/videoio/videoio.hpp> // for cv::VideoCapture
#endif

/*!
  This example allows to record a video from a camera.
  It only requires that ViSP is build with OpenCV.

  Example to save an mpeg video:

    ./tutorial-video-recorder --device 0 --name myvideo.mp4

  Example to save a sequence of png images:

    ./tutorial-video-recorder --device 0 --name image%04d.png

  Example to save one image:

    ./tutorial-video-recorder --device 0 --name image.jpeg

 */
int main(int argc, const char *argv[])
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif
  std::string opt_videoname = "video-recorded.mpg";
  int opt_device = 0;

  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--device" && i + 1 < argc) {
      opt_device = atoi(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--recorded-name" && i + 1 < argc) {
      opt_videoname = std::string(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      std::cout << "\nUsage: " << argv[0]
        << " [--device <device number>] [--recorded-name <video name>] [--help][-h]\n"
        << std::endl;
      return EXIT_SUCCESS;
    }
  }

  std::cout << "Use device: " << opt_device << std::endl;
  std::cout << "Record video in: " << opt_videoname << std::endl;

  try {
    // vpImage<vpRGBa> I; // for color images
    vpImage<unsigned char> I; // for gray images

#if defined(VISP_HAVE_V4L2)
    std::cout << "Use v4l2 grabber..." << std::endl;
    vpV4l2Grabber g;
    std::ostringstream device;
    device << "/dev/video" << opt_device;
    g.setDevice(device.str());
    g.setScale(1); // Acquire full resolution images
    g.open(I);
    g.acquire(I);
#elif defined(VISP_HAVE_OPENCV) && \
    (((VISP_HAVE_OPENCV_VERSION < 0x030000) && defined(HAVE_OPENCV_HIGHGUI)) || \
     ((VISP_HAVE_OPENCV_VERSION >= 0x030000) && defined(HAVE_OPENCV_VIDEOIO)))
    std::cout << "Use OpenCV grabber..." << std::endl;
    cv::VideoCapture g(opt_device);
    if (!g.isOpened()) { // check if we succeeded
      std::cout << "Failed to open the camera" << std::endl;
      return EXIT_FAILURE;
    }
    cv::Mat frame;
    g >> frame; // get a new frame from camera
    vpImageConvert::convert(frame, I);
#endif

    std::cout << "Image size: " << I.getWidth() << " " << I.getHeight() << std::endl;

#if defined(VISP_HAVE_DISPLAY)
    vpDisplay *d = vpDisplayFactory::allocateDisplay(I, 0, 0, "Camera view");
#endif
    vpVideoWriter writer;

#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_VIDEOIO)
#if (VISP_HAVE_OPENCV_VERSION >= 0x030000)
    writer.setCodec(cv::VideoWriter::fourcc('P', 'I', 'M', '1')); // MPEG-1 codec
#else
    writer.setCodec(CV_FOURCC('P', 'I', 'M', '1'));
#endif
#endif
    writer.setFileName(opt_videoname);
    writer.open(I);
    bool recording = false;

    for (;;) {
#if defined(VISP_HAVE_V4L2)
      g.acquire(I);
#elif defined(VISP_HAVE_OPENCV) && \
    (((VISP_HAVE_OPENCV_VERSION < 0x030000) && defined(HAVE_OPENCV_HIGHGUI)) || \
     ((VISP_HAVE_OPENCV_VERSION >= 0x030000) && defined(HAVE_OPENCV_VIDEOIO)))
      g >> frame;
      vpImageConvert::convert(frame, I);
#endif
      vpDisplay::display(I);
      if (recording == false) {
        vpDisplay::displayText(I, 10, 10, "A click to start recording", vpColor::green);
        if (vpDisplay::getClick(I, false))
          recording = true;
      }
      else {
        writer.saveFrame(I);
        vpDisplay::displayText(I, 10, 10, "Recording: A click to stop and exit", vpColor::red);
        if (vpDisplay::getClick(I, false))
          break;
      }

      vpDisplay::flush(I);
    }
    std::cout << "The video was recorded in \"" << opt_videoname << "\"" << std::endl;
#ifdef VISP_HAVE_DISPLAY
    delete d;
#endif
  }
  catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }
  return EXIT_SUCCESS;
}

#else

int main()
{
#if !defined(VISP_HAVE_DISPLAY)
  std::cout << "Install a 3rdparty to enable display feature (X11, GDI...) and rebuild ViSP to use this tutorial." << std::endl;
#endif
#if !(defined(VISP_HAVE_V4L2) || defined(VISP_HAVE_OPENCV) && \
  (((VISP_HAVE_OPENCV_VERSION < 0x030000) && defined(HAVE_OPENCV_HIGHGUI)) || \
   ((VISP_HAVE_OPENCV_VERSION >= 0x030000) && defined(HAVE_OPENCV_VIDEOIO))))
  std::cout << "Install V4l2 or OpenCV 3rdparty and rebuild ViSP to use this tutorial." << std::endl;
#endif
}

#endif
