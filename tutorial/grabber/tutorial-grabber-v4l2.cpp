/*! \example tutorial-grabber-v4l2.cpp */
#include <visp/vpV4l2Grabber.h>
#include <visp/vpDisplayX.h>
#include <visp/vpImage.h>

int main()
{
#if defined(VISP_HAVE_V4L2) && defined(VISP_HAVE_X11)
  vpImage<unsigned char> I;

  vpV4l2Grabber g;
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
