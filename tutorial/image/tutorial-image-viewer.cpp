/*! \example tutorial-image-viewer.cpp */
#include <visp3/core/vpDisplayGDI.h>
#include <visp3/core/vpDisplayOpenCV.h>
#include <visp3/core/vpDisplayX.h>
#include <visp3/core/vpImageIo.h>
#include <visp3/core/vpImagePoint.h>

int main()
{
  try {
    vpImage<vpRGBa> I;
    vpImageIo::read(I, "lena.ppm");

#if defined(VISP_HAVE_X11)
    vpDisplayX d(I);
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI d(I);
#elif defined(VISP_HAVE_OPENCV)
    vpDisplayOpenCV d(I);
#else
    std::cout << "No image viewer is available..." << std::endl;
#endif
    vpDisplay::setTitle(I, "Lena");
    vpDisplay::display(I);

    vpDisplay::displayRectangle(I, vpImagePoint(90,90), 70, 90, vpColor::red, false, 2);
    vpDisplay::flush(I);

    vpImage<vpRGBa> O;
    vpDisplay::getImage(I, O);

    try {
      vpImageIo::write(I, "lena-out.jpg");
      vpImageIo::write(O, "lena-out-with-overlay.jpg");
    }
    catch(...) {
      std::cout << "Cannot write the image: unsupported format..." << std::endl;
    }

    vpDisplay::getClick(I);
  }
  catch(vpException e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }
}
