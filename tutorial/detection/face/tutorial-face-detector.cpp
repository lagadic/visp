//! \example tutorial-face-detector.cpp
#include <visp3/core/vpConfig.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
//! [Include]
#include <visp3/detection/vpDetectorFace.h>
//! [Include]
#include <visp3/io/vpVideoReader.h>

int main(int argc, const char *argv[])
{
//! [Macro defined]
#if defined(HAVE_OPENCV_HIGHGUI) && defined(HAVE_OPENCV_IMGPROC) && \
  (((VISP_HAVE_OPENCV_VERSION < 0x050000) && defined(HAVE_OPENCV_OBJDETECT)) || ((VISP_HAVE_OPENCV_VERSION >= 0x050000) && defined(HAVE_OPENCV_XOBJDETECT)))
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif
  //! [Macro defined]
  try {
    //! [Default settings]
    std::string opt_face_cascade_name = "./haarcascade_frontalface_alt.xml";
    std::string opt_video = "video.mp4";
    //! [Default settings]

    for (int i = 1; i < argc; i++) {
      if (std::string(argv[i]) == "--haar" && i + 1 < argc) {
        opt_face_cascade_name = std::string(argv[++i]);
      }
      else if (std::string(argv[i]) == "--video" && i + 1 < argc) {
        opt_video = std::string(argv[++i]);
      }
      else if ((std::string(argv[i]) == "--help") || (std::string(argv[i]) == "-h")) {
        std::cout << "Usage: " << argv[0]
          << " [--haar <haarcascade xml filename>]"
          << " [--video <input video file>]"
          << " [--help] [-h]" << std::endl;
        return EXIT_SUCCESS;
      }
    }

    vpImage<unsigned char> I;

    vpVideoReader g;
    g.setFileName(opt_video);
    g.open(I);

#if defined(VISP_HAVE_X11)
    vpDisplayX d(I);
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI d(I);
#elif defined(HAVE_OPENCV_HIGHGUI)
    vpDisplayOpenCV d(I);
#endif
    vpDisplay::setTitle(I, "ViSP viewer");

    //! [Face detector construction]
    vpDetectorFace face_detector;
    //! [Face detector construction]
    //! [Face detector setting]
    face_detector.setCascadeClassifierFile(opt_face_cascade_name);
    //! [Face detector setting]

    bool exit_requested = false;
    while (!g.end() && !exit_requested) {
      g.acquire(I);

      vpDisplay::display(I);
      //! [Face detection]
      bool face_found = face_detector.detect(I);
      //! [Face detection]

      if (face_found) {
        std::ostringstream text;
        //! [Get number faces]
        text << "Found " << face_detector.getNbObjects() << " face(s)";
        //! [Get number faces]
        vpDisplay::displayText(I, 10, 10, text.str(), vpColor::red);
        //! [Get face characteristics]
        for (size_t i = 0; i < face_detector.getNbObjects(); i++) {
          vpRect bbox = face_detector.getBBox(i);
          vpDisplay::displayRectangle(I, bbox, vpColor::green, false, 4);
          vpDisplay::displayText(I, (int)bbox.getTop() - 10, (int)bbox.getLeft(),
                                 "Message: \"" + face_detector.getMessage(i) + "\"", vpColor::red);
        }
        //! [Get face characteristics]
      }
      vpDisplay::displayText(I, (int)I.getHeight() - 25, 10, "Click to quit...", vpColor::red);
      vpDisplay::flush(I);
      if (vpDisplay::getClick(I, false)) // a click to exit
        exit_requested = true;
    }
    if (!exit_requested)
      vpDisplay::getClick(I);
  }
  catch (const vpException &e) {
    std::cout << e.getMessage() << std::endl;
  }
#else

#if !defined(HAVE_OPENCV_HIGHGUI)
  std::cout << "This tutorial needs OpenCV highgui module that is missing." << std::endl;
#endif
#if !defined(HAVE_OPENCV_IMGPROC)
  std::cout << "This tutorial needs OpenCV imgproc module that is missing." << std::endl;
#endif
#if (VISP_HAVE_OPENCV_VERSION < 0x050000) && !defined(HAVE_OPENCV_OBJDETECT)
  std::cout << "This tutorial needs OpenCV objdetect module that is missing." << std::endl;
#endif
#if ((VISP_HAVE_OPENCV_VERSION >= 0x050000) && !defined(HAVE_OPENCV_XOBJDETECT))
  std::cout << "This tutorial needs OpenCV xobjdetect module that is missing." << std::endl;
#endif

  (void)argc;
  (void)argv;
#endif
}
