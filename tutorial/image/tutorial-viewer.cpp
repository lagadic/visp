//! \example tutorial-viewer.cpp
#include <visp3/core/vpConfig.h>
//! [Include display]
#include <visp3/gui/vpDisplayD3D.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
//! [Include display]
//! [Include io]
#include <visp3/io/vpImageIo.h>
//! [Include io]

int main(int argc, char **argv)
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif
  if (argc != 2) {
    printf("Usage: %s <image name.[pgm,ppm,jpeg,png,tiff,bmp,ras,jp2]>\n", argv[0]);
    return EXIT_FAILURE;
  }

  //! [vpImage construction]
  vpImage<vpRGBa> I;
  //! [vpImage construction]

  //! [vpImage reading]
  try {
    vpImageIo::read(I, argv[1]);
  }
  catch (...) {
    std::cout << "Cannot read image \"" << argv[1] << "\"" << std::endl;
    return EXIT_FAILURE;
  }
  //! [vpImage reading]

  try {
//! [vpDisplay construction]
#if defined(VISP_HAVE_X11)
    vpDisplayX d(I, vpDisplay::SCALE_AUTO);
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI d(I, vpDisplay::SCALE_AUTO);
#elif defined(HAVE_OPENCV_HIGHGUI)
    vpDisplayOpenCV d(I, vpDisplay::SCALE_AUTO);
#elif defined(VISP_HAVE_GTK)
    vpDisplayGTK d(I, vpDisplay::SCALE_AUTO);
#elif defined(VISP_HAVE_D3D9)
    vpDisplayD3D d(I, vpDisplay::SCALE_AUTO);
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
  catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }
}
