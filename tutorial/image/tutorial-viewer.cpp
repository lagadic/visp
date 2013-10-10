/*! \example tutorial-viewer.cpp */
#include <visp/vpDisplayD3D.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayGTK.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpImageIo.h>

int main(int argc, char** argv)
{
  if(argc != 2) {
    printf( "Usage: %s <image name.[pgm,ppm,jpeg,png,tiff,bmp,ras,jp2]>\n", argv[0] );
    return -1;
  }

  vpImage<vpRGBa> I;

  try {
    vpImageIo::read(I, argv[1]);
  }
  catch(...) {
    std::cout << "Cannot read image \"" << argv[1] << "\"" << std::endl;
    return -1;
  }

  try {
#if defined(VISP_HAVE_X11)
    vpDisplayX d(I);
#elif defined(VISP_HAVE_OPENCV)
    vpDisplayOpenCV d(I);
#elif defined(VISP_HAVE_GTK)
    vpDisplayGTK d(I);
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI d(I);
#elif defined(VISP_HAVE_D3D9)
    vpDisplayD3d d(I);
#else
    std::cout << "No image viewer is available..." << std::endl;
#endif
    vpDisplay::setTitle(I, "My image");
    vpDisplay::display(I);
    vpDisplay::flush(I);
    std::cout << "A click to quit..." << std::endl;
    vpDisplay::getClick(I);
  }
  catch(vpException e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }
}
