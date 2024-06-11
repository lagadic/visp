//! \example tutorial-video-reader.cpp
#include <visp3/core/vpConfig.h>
#include <visp3/gui/vpDisplayFactory.h>
//! [Include]
#include <visp3/core/vpTime.h>
#include <visp3/io/vpVideoReader.h>
//! [Include]

/*!
   This example allows to read and display a video from a file.
   It only requires that ViSP is build with OpenCV.

   Example: ./tutorial-video-reader --name video.mp4
 */
int main(int argc, char **argv)
{
#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_HIGHGUI) && defined(HAVE_OPENCV_VIDEOIO)
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif
  try {
    std::string videoname = "video.mp4";

    for (int i = 0; i < argc; i++) {
      if (std::string(argv[i]) == "--name")
        videoname = std::string(argv[i + 1]);
      else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
        std::cout << "\nUsage: " << argv[0] << " [--name <video name> (default: " << videoname << ")]"
          << " [--help] [-h]\n"
          << std::endl;
        return EXIT_SUCCESS;
      }
    }

    vpImage<unsigned char> I;
    //! [vpVideoReader construction]
    vpVideoReader g;
    //! [vpVideoReader construction]
    //! [vpVideoReader setting]
    g.setFileName(videoname);
    //! [vpVideoReader setting]
    //! [vpVideoReader open]
    g.open(I);
    //! [vpVideoReader open]
    std::cout << "Video name     : " << videoname << std::endl;
    std::cout << "Video framerate: " << g.getFramerate() << "Hz" << std::endl;
    std::cout << "Video dimension: " << I.getWidth() << " " << I.getHeight() << std::endl;

#ifdef VISP_HAVE_DISPLAY
    vpDisplay *d = vpDisplayFactory::displayFactory(I);
#else
    std::cout << "No image viewer is available..." << std::endl;
#endif
    vpDisplay::setTitle(I, "Video reader");

    unsigned cpt = 1;
    bool quit = false;
    //! [vpVideoReader while loop]
    while ((!g.end()) && (!quit)) {
      //! [vpVideoReader while loop]
      //! [vpVideoReader loop start time]
      double t = vpTime::measureTimeMs();
      //! [vpVideoReader loop start time]
      //! [vpVideoReader acquire]
      g.acquire(I);
      //! [vpVideoReader acquire]
      vpDisplay::display(I);
      vpDisplay::displayText(I, 20, 20, "Click to quit", vpColor::red);
      std::stringstream ss;
      ss << "Frame: " << ++cpt;
      std::cout << "Read " << ss.str() << std::endl;
      vpDisplay::displayText(I, 40, 20, ss.str(), vpColor::red);
      vpDisplay::flush(I);
      if (vpDisplay::getClick(I, false)) {
        quit = true;
      }
      //! [vpVideoReader loop rate]
      vpTime::wait(t, 1000. / g.getFramerate());
      //! [vpVideoReader loop rate]
    }
    if (!quit) {
      std::cout << "End of video was reached" << std::endl;
    }
#ifdef VISP_HAVE_DISPLAY
    delete d;
#endif
  }
  catch (const vpException &e) {
    std::cout << e.getMessage() << std::endl;
  }
#else
  (void)argc;
  (void)argv;
  std::cout << "Install OpenCV and rebuild ViSP to use this example." << std::endl;
#endif
  return EXIT_SUCCESS;
}
