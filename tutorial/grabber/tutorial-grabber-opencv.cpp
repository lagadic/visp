/*! \example tutorial-grabber-opencv.cpp */
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/sensor/vpOpenCVGrabber.h>

int main()
{
#if defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION < 0x020408)
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
