/*! \example tutorial-grabber-video.cpp */
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpDisplayX.h>
#include <visp/vpTime.h>
#include <visp/vpVideoReader.h>

int main()
{
  try {
    vpImage<unsigned char> I;

    vpVideoReader g;
    g.setFileName("./video.mpg");
    g.open(I);

    std::cout << "video framerate: " << g.getFramerate() << "Hz" << std::endl;
    std::cout << "video dimension: " << I.getWidth() << " " << I.getHeight() << std::endl;

#ifdef VISP_HAVE_X11
    vpDisplayX d(I);
#elif VISP_HAVE_GDI
    vpDisplayGDI d(I);
#elif VISP_HAVE_OPENCV
    vpDisplayOpenCV d(I);
#else
    std::cout << "No image viewer is available..." << std::endl;
#endif
    vpDisplay::setTitle(I, "Video grabber");
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
}
