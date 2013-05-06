/*! \example tutorial-viewer.cpp */
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayX.h>
#include <visp/vpImageIo.h>

int main(int argc, char** argv )
{
  if(argc != 2) {
    printf( "Usage: viewer <image name.[pgm,ppm,jpeg,png]>\n" );
    return -1;
  }

  vpImage<vpRGBa> I;

  try {
    vpImageIo::read(I, argv[1]);
  }
  catch(...) {
    std::cout << "Cannot read image \"" << argv[1] << "\"" << std::endl;
  }

#if defined(VISP_HAVE_X11)
  vpDisplayX d(I);
#elif defined(VISP_HAVE_GDI)
  vpDisplayGDI d(I);
#else
  std::cout << "No image viewer is available..." << std::endl;
#endif
  vpDisplay::setTitle(I, "My image");
  vpDisplay::display(I);
  vpDisplay::flush(I);
  std::cout << "A click to quit..." << std::endl;
  vpDisplay::getClick(I);
}
