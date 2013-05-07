/*! \example tutorial-grabber-video.cpp */
#include <visp/vpDisplayX.h>
#include <visp/vpVideoReader.h>

int main()
{
#ifdef VISP_HAVE_FFMPEG
  vpImage<unsigned char> I;

  vpVideoReader g;
  g.setFileName("./video.mpg");
  g.open(I);

  std::cout << "video framerate: " << g.getFramerate() << "Hz" << std::endl;
  std::cout << "video dimension: " << I.getWidth() << " " << I.getHeight() << std::endl;

#ifdef VISP_HAVE_X11
  vpDisplayX d(I);
#else
  std::cout << "No image viewer is available..." << std::endl;
#endif
  vpDisplay::setTitle(I, "Video grabber");
  while(1) {
    g.acquire(I);
    vpDisplay::display(I);
    vpDisplay::flush(I);
    if (vpDisplay::getClick(I, false)) break;
  }
#endif
}
