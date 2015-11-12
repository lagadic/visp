//! \example tutorial-viewer.cpp
//! [Include display]
#include <visp3/gui/vpDisplayD3D.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpDisplayOpenCV.h>
//! [Include display]
//! [Include io]
#include <visp3/io/vpImageIo.h>
//! [Include io]

int main(int argc, char** argv)
{
  if(argc != 2) {
    printf( "Usage: %s <image name.[pgm,ppm,jpeg,png,tiff,bmp,ras,jp2]>\n", argv[0] );
    return -1;
  }

  //! [vpImage construction]
  vpImage<vpRGBa> I;
  //! [vpImage construction]

  //! [vpImage reading]
  try {
    vpImageIo::read(I, argv[1]);
  }
  catch(...) {
    std::cout << "Cannot read image \"" << argv[1] << "\"" << std::endl;
    return -1;
  }
  //! [vpImage reading]

  try {
    //! [vpDisplay construction]
#if defined(VISP_HAVE_X11)
    vpDisplayX d(I);
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI d(I);
#elif defined(VISP_HAVE_OPENCV)
    vpDisplayOpenCV d(I);
#elif defined(VISP_HAVE_GTK)
    vpDisplayGTK d(I);
#elif defined(VISP_HAVE_D3D9)
    vpDisplayD3d d(I);
#else
    std::cout << "No image viewer is available..." << std::endl;
#endif
    //! [vpDisplay construction]
    //! [vpDisplay set title]
    vpDisplay::setTitle(I, "My image");
    //! [vpDisplay set title]
    //! [vpDisplay display]
    vpDisplay::display(I);
    vpDisplay::flush(I);
    //! [vpDisplay display]
    std::cout << "A click to quit..." << std::endl;
    //! [vpDisplay get click]
    vpDisplay::getClick(I);
    //! [vpDisplay get click]
  }
  catch(vpException e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }
}
