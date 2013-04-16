/*! \example tutorial-grabber-CMU1394.cpp */
#include <visp/vp1394CMUGrabber.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpImage.h>

int main()
{
#if defined(VISP_HAVE_CMU1394) && defined(VISP_HAVE_GDI)
  vpImage<unsigned char> I;

  vp1394CMUGrabber g;
  g.setVideoMode(0, 1); // 640x480 MONO8
  g.setAutoShutter();
  g.setAutoGain();
  g.setFramerate(4); // 30 fps
  g.open(I);
  g.acquire(I);
  std::cout << I.getWidth() << " " << I.getHeight() << std::endl;
  vpDisplayGDI d(I);

  while(1) {
    g.acquire(I);
    vpDisplay::display(I);
    vpDisplay::flush(I);
    if (vpDisplay::getClick(I, false)) break;
  }
#endif
}
