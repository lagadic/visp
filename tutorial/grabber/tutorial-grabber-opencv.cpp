/*! \example tutorial-grabber-opencv.cpp */
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpOpenCVGrabber.h>

int main()
{
#ifdef VISP_HAVE_OPENCV
  try {
    vpImage<unsigned char> I;

    vpOpenCVGrabber g;
    g.open(I);

    std::cout << "Image size: " << I.getWidth() << " " << I.getHeight() << std::endl;
    vpDisplayOpenCV d(I);

    while(1) {
      g.acquire(I);
      vpDisplay::display(I);
      vpDisplay::flush(I);
      if (vpDisplay::getClick(I, false))
        break;
    }
  }
  catch(vpException e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }
#endif
}
