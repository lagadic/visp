//! \example tutorial-face-detector.cpp
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
//! [Include]
#include <visp3/detection/vpDetectorFace.h>
//! [Include]
#include <visp3/io/vpVideoReader.h>

int main(int argc, const char* argv[])
{
//! [Macro defined]
#if defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020200)
  //! [Macro defined]
  try {
    //! [Default settings]
    std::string opt_face_cascade_name = "./haarcascade_frontalface_alt.xml";
    std::string opt_video = "video.mpeg";
    //! [Default settings]

    for (int i=0; i<argc; i++) {
      if (std::string(argv[i]) == "--haar")
        opt_face_cascade_name = std::string(argv[i+1]);
      else if (std::string(argv[i]) == "--video")
        opt_video = std::string(argv[i+1]);
      else if (std::string(argv[i]) == "--help") {
        std::cout << "Usage: " << argv[0] << " [--haar <haarcascade xml filename>] [--video <input video file>] [--help]" << std::endl;
        return 0;
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
#elif defined(VISP_HAVE_OPENCV)
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
    while( ! g.end() && ! exit_requested) {
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
        for(size_t i=0; i < face_detector.getNbObjects(); i++) {
          std::vector<vpImagePoint> p = face_detector.getPolygon(i);
          vpRect bbox = face_detector.getBBox(i);
          vpDisplay::displayRectangle(I, bbox, vpColor::green, false, 4);
          vpDisplay::displayText(I, (int)bbox.getTop()-10, (int)bbox.getLeft(),
                                 "Message: \"" + face_detector.getMessage(i) + "\"", vpColor::red);
        }
        //! [Get face characteristics]
      }
      vpDisplay::displayText(I, (int)I.getHeight()-25, 10, "Click to quit...", vpColor::red);
      vpDisplay::flush(I);
      if (vpDisplay::getClick(I, false)) // a click to exit
        exit_requested = true;
    }
    if (! exit_requested)
      vpDisplay::getClick(I);
  }
  catch(vpException &e) {
    std::cout << e.getMessage() << std::endl;
  }
#else
  (void)argc;
  (void)argv;
#endif
}
