/*! \example tutorial-video-reader.cpp */
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpDisplayX.h>
#include <visp/vpTime.h>
#include <visp/vpVideoReader.h>

/*!
 This example allows to read and display a video from a file.
 It only requires that ViSP is build with OpenCV or ffmpeg.

 Example: ./tutorial-video-reader --name myvideo.mpeg
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

    vpVideoReader g;
    g.setFileName(videoname);
    g.open(I);
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
    while (! g.end() ) {
      double t = vpTime::measureTimeMs();
      g.acquire(I);
      vpDisplay::display(I);
      vpDisplay::flush(I);
      if (vpDisplay::getClick(I, false)) break;
      vpTime::wait(t, 1000. / g.getFramerate());
    }
  }
  catch(vpException e) {
    std::cout << e.getMessage() << std::endl;
  }
#else
  std::cout << "Install OpenCV or ffmpeg and rebuild ViSP to use this example." << std::endl;
#endif
}
