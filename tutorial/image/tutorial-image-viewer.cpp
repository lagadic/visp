/*! \example tutorial-image-viewer.cpp */
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/core/vpImagePoint.h>

int main()
{
  try {
    vpImage<vpRGBa> I;
    vpImageIo::read(I, "monkey.ppm");

#if defined(VISP_HAVE_X11)
    vpDisplayX d(I);
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI d(I);
#elif defined(VISP_HAVE_OPENCV)
    vpDisplayOpenCV d(I);
#else
    std::cout << "No image viewer is available..." << std::endl;
#endif
    vpDisplay::setTitle(I, "Monkey");
    vpDisplay::display(I);

    vpDisplay::displayRectangle(I, vpImagePoint(90,90), 70, 90, vpColor::red, false, 2);
    vpDisplay::flush(I);

    vpImage<vpRGBa> O;
    vpDisplay::getImage(I, O);

    try {
      vpImageIo::write(I, "monkey-out.jpg");
      vpImageIo::write(O, "monkey-out-with-overlay.jpg");
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
