/*! \example tutorial-grabber-1394.cpp */
#include <visp/vp1394TwoGrabber.h>
#include <visp/vpDisplayX.h>
#include <visp/vpImage.h>

int main()
{
#if defined(VISP_HAVE_DC1394_2) && defined(VISP_HAVE_X11)
  vpImage<vpRGBa> I;

  vp1394TwoGrabber g;
  g.open(I);
  g.acquire(I);
  std::cout << I.getWidth() << " " << I.getHeight() << std::endl;
  vpDisplayX d(I);

  while(1) {
    g.acquire(I);
    vpDisplay::display(I);
    vpDisplay::flush(I);
    if (vpDisplay::getClick(I, false)) break;
  }
#endif
}
