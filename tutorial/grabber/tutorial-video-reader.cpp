//! \example tutorial-video-reader.cpp
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
//! [Include]
#include <visp3/core/vpTime.h>
#include <visp3/io/vpVideoReader.h>
//! [Include]

/*!
 This example allows to read and display a video from a file.
 It only requires that ViSP is build with OpenCV or ffmpeg.

 Example: ./tutorial-video-reader --name video.mpg
 */
int main(int argc, char** argv)
{
#if defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100) || defined(VISP_HAVE_FFMPEG)
  try {
    std::string videoname = "video.mpg";

    for (int i=0; i<argc; i++) {
      if (std::string(argv[i]) == "--name")
        videoname = std::string(argv[i+1]);
      else if (std::string(argv[i]) == "--help") {
        std::cout << "\nUsage: " << argv[0] << " [--name <video name>] [--help]\n" << std::endl;
        return 0;
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
    std::cout << "video name: " << videoname << std::endl;
    std::cout << "video framerate: " << g.getFramerate() << "Hz" << std::endl;
    std::cout << "video dimension: " << I.getWidth() << " " << I.getHeight() << std::endl;

#ifdef VISP_HAVE_X11
    vpDisplayX d(I);
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI d(I);
#elif defined(VISP_HAVE_OPENCV)
    vpDisplayOpenCV d(I);
#else
    std::cout << "No image viewer is available..." << std::endl;
#endif
    vpDisplay::setTitle(I, "Video reader");
    //! [vpVideoReader while loop]
    while (! g.end() ) {
      //! [vpVideoReader while loop]
      //! [vpVideoReader loop start time]
      double t = vpTime::measureTimeMs();
      //! [vpVideoReader loop start time]
      //! [vpVideoReader acquire]
      g.acquire(I);
      //! [vpVideoReader acquire]
      vpDisplay::display(I);
      vpDisplay::flush(I);
      if (vpDisplay::getClick(I, false)) break;
      //! [vpVideoReader loop rate]
      vpTime::wait(t, 1000. / g.getFramerate());
      //! [vpVideoReader loop rate]
    }
  }
  catch(vpException &e) {
    std::cout << e.getMessage() << std::endl;
  }
#else
  (void)argc;
  (void)argv;
  std::cout << "Install OpenCV or ffmpeg and rebuild ViSP to use this example." << std::endl;
#endif
}
