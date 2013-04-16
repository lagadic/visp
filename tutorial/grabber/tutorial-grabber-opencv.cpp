/*! \example tutorial-grabber-opencv.cpp */
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpOpenCVGrabber.h>

int main()
{
#ifdef VISP_HAVE_OPENCV
  vpImage<unsigned char> I;

  vpOpenCVGrabber g;
  g.open(I);
  g.acquire(I);
  std::cout << I.getWidth() << " " << I.getHeight() << std::endl;
  vpDisplayOpenCV d(I);

  while(1) {
    g.acquire(I);
    vpDisplay::display(I);
    vpDisplay::flush(I);
    if (vpDisplay::getClick(I, false)) break;
  }
#endif
}
